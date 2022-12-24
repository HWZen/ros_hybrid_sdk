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
#include <google/protobuf/util/json_util.h>

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


    awaitable<DispatchResultCode> dispatch(ref_client &client);

    void toDefaultAgent(const ref_client &client);

    void toNewAgent(const ref_client &client);

    void sendSocketor(int pip, const ref_client &client);

    awaitable<void> listen();

    DefaultAgentManager defaultAgent;

    Log logger{ "DispatchServer", LogFlag::CONSOLE_LOGGER };

    int defaultAgentPip{};

    int preBootAgentPip{};

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

void DispatchServer::Impl::sendSocketor(int pip, const ref_client &client)
{
    auto fd = client->native_handle();
    send_fd(pip, fd);
}

void DispatchServer::Impl::init(uint16_t port)
{
    int defaultAgentPipFd[2];
    int preBootAgentPidFd[2];

    if (socketpair(PF_UNIX, SOCK_DGRAM, 0, defaultAgentPipFd) < 0){
        logger.error("DefaultAgent socketpair error");
        throw SDKException("DefaultAgent socketpair error");
    }

    if(socketpair(PF_UNIX, SOCK_DGRAM, 0, preBootAgentPidFd) < 0){
        logger.error("PreBootAgent socketpair error");
        throw SDKException("PreBootAgent socketpair error");
    }


    /*****************
     * init DefaultAgentManager
     ****************/

    auto defaultAgentPid = fork();

    if (defaultAgentPid < 0)
        throw SDKException("fork failed");

    // child
    if (defaultAgentPid == 0){
        close(defaultAgentPipFd[1]);
        {
            defaultAgent.setSocketPipe(defaultAgentPipFd[0]);
            defaultAgent.MAIN();
        }
        exit(0);
    }
    // parent
    close(defaultAgentPipFd[0]);
    Impl::defaultAgentPip = defaultAgentPipFd[1];

    /*****************
     * init PreBootAgentManager
     ****************/
    auto preBootPid = fork();

    if (preBootPid < 0)
        throw SDKException("fork failed");

    // child
    if (preBootPid == 0){
        close(preBootAgentPidFd[1]);
        {
            Agent agent(preBootAgentPidFd[0]);
            agent.MAIN();
        }
        exit(0);
    }
    // parent
    close(preBootAgentPidFd[0]);
    Impl::preBootAgentPip = preBootAgentPidFd[1];

    /*****************
     * init acceptor
     ****************/
    auto listen_endpoint = tcp::endpoint{tcp::v4(), port};
    acceptor = std::make_shared<tcp::acceptor>(ctx, std::move(listen_endpoint));
}

void DispatchServer::Impl::run()
{
    if (!acceptor->is_open())[[unlikely]] {
        this->logger.error("Server no init or not listen");
        throw SDKException("Server no init or not listen");
    }
    co_spawn(ctx, DispatchServer::Impl::listen(), asio::detached);
    ctx.run();
    logger.info("ctx exit");
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
                auto ref_client = make_client(std::move(client));
                auto res = co_await dispatch(ref_client);
                if (res == DispatchResultCode::FAIL)
                    logger.error("Dispatch failed");
                else if (res == DispatchResultCode::DEFAULT_AGENT)
                    toDefaultAgent(ref_client);
                else if (res == DispatchResultCode::NEW_AGENT)
                    toNewAgent(ref_client);
            }
            catch (std::exception &e) {
                this->logger.error("Catch dispatch exception: {}", e.what());
                this->logger.info("listen continue");
            }
        }, asio::detached);
    }
}



awaitable<DispatchServer::Impl::DispatchResultCode> DispatchServer::Impl::dispatch(ref_client &client)
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
        co_return DispatchResultCode::FAIL;
    }
    buff_[len] = '\0';

    /*******************************
     * parse param
     ******************************/

    // protobuf style parse
    if (client->agentConfig.ParseFromArray(buff_.data(), static_cast<int>(len))){
        if (client->agentConfig.node().empty()){
            logger.error("Node is empty");
            co_await client->async_write_some(asio::buffer("Node is empty"), use_nothrow_awaitable);
            co_return DispatchResultCode::FAIL;
        }
        logger.debug("parse protobuf, agent config: {}", client->agentConfig.DebugString());
        co_return client->agentConfig.node() == "default" ? DispatchResultCode::DEFAULT_AGENT : DispatchResultCode::NEW_AGENT;
    }

    // json style parse

    auto state = google::protobuf::util::JsonStringToMessage(std::string_view(buff_.data(), len), &client->agentConfig);
    if (!state.ok()){
        logger.error("Config string is not a protobuf or json : {}", state.ToString());
        co_await client->async_write_some(asio::buffer("Config string is not a protobuf or json : " + state.ToString()), use_nothrow_awaitable);
        co_return DispatchResultCode::FAIL;
    }

    auto &agentConfig = client->agentConfig;

    if (agentConfig.node().empty()){
        logger.error("node is empty");
        co_await client->async_write_some(asio::buffer("node is empty"), use_nothrow_awaitable);
        co_return DispatchResultCode::FAIL;
    }

    if (agentConfig.has_delimiter_str())
        agentConfig.set_delimiter(agentConfig.delimiter_str());
    if (!agentConfig.has_delimiter())
        agentConfig.set_delimiter(HYBRID_DELIMITER);
    if (!agentConfig.has_is_protobuf())
        agentConfig.set_is_protobuf(false);
    if (!agentConfig.has_log_level())
        agentConfig.set_log_level(2);

    logger.debug("parse json, agent config: {}", client->agentConfig.DebugString());


    /*******************************
     * parse param over
     ******************************/

    const auto &node = client->agentConfig.node();

    if (node == "default")
        co_return DispatchResultCode::DEFAULT_AGENT;
    else
        co_return DispatchResultCode::NEW_AGENT;
}



void DispatchServer::Impl::toDefaultAgent(const ref_client &client)
{
    logger.debug("in {}", __FUNCTION__);
    sendSocketor(defaultAgentPip, client);
    auto buf = client->agentConfig.SerializeAsString();
    write(defaultAgentPip, buf.data(), buf.size());
}

void DispatchServer::Impl::toNewAgent(const ref_client &client)
{
    logger.debug("in {}", __FUNCTION__);
    sendSocketor(preBootAgentPip, client);
    auto buf = client->agentConfig.SerializeAsString();
    write(preBootAgentPip, buf.data(), buf.size());
}

