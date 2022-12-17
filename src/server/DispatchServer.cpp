//
// Created by HWZ on 2022/11/2.
//

#include "DispatchServer.h"
#include "../SDKException.h"
#include "json.h"
#include "DefaultAgent/DefaultAgentManager.h"
#include "Agent/Agent.h"

#include "Log.h"
#include "asioHeader.h"

using namespace std::chrono_literals;
using namespace std::string_literals;



using ref_client = RefSocketor;

/**
 * DispatchServer::Imp define
 */
struct DispatchServer::Impl
{

    void init(uint16_t port = 5150);

    void run();


    enum class DispatchResultCode : int
    {
        FAIL = -1,
        DEFAULT_AGENT = 0,
        NEW_AGENT = 1
    };

    int send_fd(int pipe_fd, SOCKET fd);

    struct DispatchResult
    {
        DispatchResultCode code;
        std::string agentName;
    };


    awaitable<DispatchResult> dispatch(const ref_client &client);

    void toDefaultAgent(const ref_client &client);

    awaitable<void> toNewAgent(const ref_client &client, const std::string &agentName);

    void sendSocketor(const ref_client &client);

    awaitable<void> listen();

    DefaultAgentManager defaultAgent;

    Log logger{ "DispatchServer", LogFlag::CONSOLE_LOGGER };

    int pip{};

    std::shared_ptr<tcp::acceptor> acceptor{};

    asio::io_context ctx{};

};




void DispatchServer::init(uint16_t port)
{
    implPtr = new Impl;
    implPtr->init(port);
}
void DispatchServer::run()
{
    implPtr->run();
}
DispatchServer::~DispatchServer()
{
    delete implPtr;
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
        logger.error("send fd error, code: {}, reason: {}", errCode, reason);
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
        throw SDKException("socketpair error");
    }


    /*****************
     * init DefaultAgentManager
     ****************/

    auto pid = fork();

    if (pid < 0)
        throw SDKException("fork failed");

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
        throw SDKException("Server no init or not listen");
    }
    co_spawn(ctx, DispatchServer::Impl::listen(), asio::detached);
    ctx.run();
}

awaitable<void> DispatchServer::Impl::listen()
{
    logger.info("in DispatchServer::Impl::listen");
    for(;;){
        auto [ec, client] = co_await acceptor->async_accept(use_nothrow_awaitable);
        if (ec) [[unlikely]] {
            logger.error("Accept error: {}", ec.message());
            break;
        }

        auto executor = client.get_executor();
        co_spawn(executor,[this, client = std::move(client)]() mutable -> awaitable<void> {
            try{
                auto ref_client = std::make_shared<decltype(client)>(std::move(client));
                auto [res, agentName] = co_await dispatch(ref_client);
                if (res == DispatchResultCode::FAIL)
                    logger.error("Dispatch failed");
                else if (res == DispatchResultCode::DEFAULT_AGENT)
                    toDefaultAgent(ref_client);
                else if (res == DispatchResultCode::NEW_AGENT) {
                    auto pid = fork();
                    if (pid == -1) [[unlikely]] {
                        auto errCode = errno;
                        auto errMsg = strerror(errCode);
                        logger.error("Fork failed, error code: {}, error message: {}", errCode, errMsg);
                        throw SDKException("Fork failed");
                    }
                    if (pid == 0)  // sub process
                        co_return co_await toNewAgent(ref_client, agentName);
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



awaitable<DispatchServer::Impl::DispatchResult> DispatchServer::Impl::dispatch(const ref_client &client)
{

    if (!client->is_open()) {
        logger.error("Accept error");
        throw SDKException("Accept error");
    }
    logger.info("Accept a client, from: {}", client->remote_endpoint().address().to_string());

    std::array<char, 4096> buff_{};
    auto [ec, len] = co_await (*client).async_read_some(asio::buffer(buff_), use_nothrow_awaitable);
    if (ec){
        logger.error("Read error: {}", ec.message());
        co_return DispatchResult{DispatchResultCode::FAIL, ""};
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
        co_return DispatchResult{DispatchResultCode::FAIL, ""};
    }

    if (!json.HasMember("node") || !json["node"].IsString()) {
        logger.error("can not found json key: node");
        co_await client->async_write_some(asio::buffer("can not found json key: node"), use_nothrow_awaitable);
        co_return DispatchResult{DispatchResultCode::FAIL, ""};
    }

    /*******************************
     * parse param over
     ******************************/

    auto node = std::string{json["node"].GetString()};

    if (node == "default")
        co_return DispatchResult{DispatchResultCode::DEFAULT_AGENT, ""};
    else
        co_return DispatchResult{DispatchResultCode::NEW_AGENT, std::move(node)};
}



void DispatchServer::Impl::toDefaultAgent(const ref_client &client)
{
    logger.debug("in {}", __FUNCTION__);
    sendSocketor(client);
}

awaitable<void> DispatchServer::Impl::toNewAgent(const ref_client &client, const std::string &agentName)
{
    logger.debug("in {}", __FUNCTION__);
    // close parent acceptor
    acceptor->close();

    Agent agent(client, agentName);

    co_await agent.MAIN();
    co_return ;
}

