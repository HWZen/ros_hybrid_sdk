//
// Created by HWZ on 2022/11/2.
//

#include "DispatchServer.h"
#include <ros/console.h>
#include <sstl/ref_ptr.h>
#include "json.h"
#include "DefaultAgent.h"
#include "Agent.h"
#include "RefSocketor.h"
#include "Log.h"


#include <asio.hpp>
#include <asio/experimental/as_tuple.hpp>
#include <asio/experimental/awaitable_operators.hpp>
using asio::awaitable;
using asio::buffer;
using asio::co_spawn;
using asio::ip::tcp;
namespace this_coro = asio::this_coro;
using namespace asio::experimental::awaitable_operators;
using std::chrono::steady_clock;
using namespace std::chrono_literals;
using namespace std::string_literals;



using ref_client = RefSocketor;

constexpr auto use_nothrow_awaitable = asio::experimental::as_tuple(asio::use_awaitable);

/**
 * DispatchServer::Imp define
 */
struct DispatchServer::Impl
{

    void init(uint16_t port = 5150);

    void run();


    enum class DispatchResult : int
    {
        FAIL = -1,
        DEFAULT_AGENT = 0,
        NEW_AGENT = 1
    };

    int send_fd(int pipe_fd, SOCKET fd);

    awaitable<int> dispatch(const ref_client &client);

    int toDefaultAgent(const ref_client &client);

    int toNewAgent(const ref_client &client);

    void sendSocketor(const ref_client &client);

    awaitable<void> listen();

    DefaultAgent defaultAgent;

    Log logger{ "DispatchServer", LogFlag::CONSOLE_LOGGER };

    int pip{};

    std::shared_ptr<tcp::acceptor> acceptor{};

    asio::io_context ctx{};

};




void DispatchServer::init(uint16_t port)
{
    impl_ = new Impl;
    impl_->init(port);
}
void DispatchServer::run()
{
    impl_->run();
}
DispatchServer::~DispatchServer()
{
    delete impl_;
}


/**
 * DispatchServer::Imp implement
 */
int DispatchServer::Impl::send_fd(int pipe_fd, SOCKET fd)
{
    iovec iov[1];
    msghdr msg{};
    char buff[0];

    iov[0].iov_base = buff;
    iov[0].iov_len = 1;

    msg.msg_name = nullptr;
    msg.msg_namelen = 0;

    msg.msg_iov = iov;
    msg.msg_iovlen = 1;

    char cmBuf[CMSG_LEN(sizeof(fd))];
    cmsghdr &cm = *(new(cmBuf) cmsghdr{CMSG_LEN(sizeof(pipe_fd)), SOL_SOCKET, SCM_RIGHTS});
    *(int *)cm.__cmsg_data = fd;

    msg.msg_control = &cm;
    msg.msg_controllen = CMSG_LEN(sizeof(pipe_fd));

    auto res = sendmsg(pipe_fd, &msg, 0); // 发送描述符
    if (res == -1) {
        auto errCode = errno;
        auto reason = hstrerror(errCode);
        logger.error("send error, code: {}, reason: {}", errCode, reason);
        return errCode;
    }
    return 0;
}

void DispatchServer::Impl::sendSocketor(const ref_client &client)
{
    auto fd = client->native_handle();
    send_fd(pip, fd);
}

void DispatchServer::Impl::init(uint16_t port)
{
    int pipfd[2];

    if (socketpair(PF_UNIX, SOCK_DGRAM, 0, pipfd) < 0){
        logger.error("socketpair error");
        throw ros::Exception("socketpair error");
    }


    auto pid = fork();

    if (pid < 0)
        throw ros::Exception("fork failed");

    // child
    if (pid == 0){
        close(pipfd[1]);
        defaultAgent.setSocketPipe(pipfd[0]);
        defaultAgent.MAIN();
    }
    // parent
    else{
        close(pipfd[0]);
        auto listen_endpoint = tcp::endpoint{tcp::v4(), port};
        acceptor = std::make_shared<tcp::acceptor>(ctx, std::move(listen_endpoint));
        pip = pipfd[1];
    }

}

void DispatchServer::Impl::run()
{
    if (!this || !acceptor->is_open())[[unlikely]] {
        this->logger.error("Server no init or not listen");
        throw ros::Exception("Server no init or not listen");
    }
    co_spawn(ctx, DispatchServer::Impl::listen(), asio::detached);
    ctx.run();
}

awaitable<void> DispatchServer::Impl::listen()
{
    logger.info("in DispatchServer::Impl::listen");
    for(;;){
        auto [ec, client] = co_await acceptor->async_accept(use_nothrow_awaitable);
        if (ec) {
            logger.error("Accept error: {}", ec.message());
            continue;
        }

        co_spawn(client.get_executor(),[this, client = std::move(client)]() mutable -> awaitable<void> {
            try{
                auto ref_client = std::make_shared<decltype(client)>(std::move(client));
                auto res = (Impl::DispatchResult) co_await dispatch(ref_client);
                if (res == DispatchResult::FAIL)
                    logger.error("Dispatch failed");
                else if (res == DispatchResult::DEFAULT_AGENT)
                    toDefaultAgent(ref_client);
                else if (res == DispatchResult::NEW_AGENT) {
                    auto pid = fork();
                    if (pid == -1) {
                        auto errCode = errno;
                        auto errMsg = strerror(errCode);
                        logger.error("Fork failed, error code: {}, error message: {}", errCode, errMsg);
                        throw ros::Exception("Fork failed");
                    }
                    if (pid == 0)  // sub process
                        toNewAgent(ref_client);
                    // parent process, continue
                }
            }
            catch (std::exception &e) {
                this->logger.error("Catch dispatch exception: {}", e.what());
                this->logger.info("listen continue");
            }
        }, asio::detached);
    }
}


awaitable<int> DispatchServer::Impl::dispatch(const ref_client &client)
{

    if (!client->is_open()) {
        logger.error("Accept error");
        throw ros::Exception("Accept error");
    }
    logger.info("Accept a client, from: {}", client->remote_endpoint().address().to_string());

    std::array<char, 4096> buff_{};
    auto [ec, len] = co_await (*client).async_read_some(asio::buffer(buff_), use_nothrow_awaitable);
    if (ec){
        logger.error("Read error: {}", ec.message());
        co_return static_cast<int>(DispatchResult::FAIL);
    }
    buff_[len] = '\0';

    /*******************************
     * parse param
     ******************************/

    JsonParse json(buff_.data());

    if (json.HasParseError()) {
        auto error_str = "Json parse failed, errorCode: "s + std::to_string(json.GetParseError()) + ", offset: "s
                         + std::to_string(json.GetErrorOffset()) + ", error: "s + rapidjson::GetParseError_En(json.GetParseError());
        logger.error("{}", error_str.c_str());
        co_await client->async_write_some(asio::buffer(error_str), use_nothrow_awaitable);
        co_return static_cast<int>(DispatchResult::FAIL);
    }

    if (!json.HasMember("node") || !json["node"].IsString()) {
        logger.error("can not found json key: node");
        co_await client->async_write_some(asio::buffer("can not found json key: node"), use_nothrow_awaitable);
        co_return static_cast<int>(DispatchResult::FAIL);
    }

    /*******************************
     * parse param over
     ******************************/

    auto node = std::string_view{json["node"].GetString()};

    if (node == "default")
        co_return static_cast<int>(DispatchResult::DEFAULT_AGENT);
    else
        co_return static_cast<int>(DispatchResult::NEW_AGENT);

}



int DispatchServer::Impl::toDefaultAgent(const ref_client &client)
{
    logger.debug("in {}", __FUNCTION__);
    sendSocketor(client);
    return static_cast<int>(DispatchResult::DEFAULT_AGENT);
}

int DispatchServer::Impl::toNewAgent(const ref_client &client)
{
    logger.debug("in {}", __FUNCTION__);
    // close parent acceptor
    acceptor->close();

    Agent agent(client);

    agent.MAIN();

}

