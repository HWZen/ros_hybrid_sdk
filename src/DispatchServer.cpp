//
// Created by HWZ on 2022/11/2.
//

#include "DispatchServer.h"
#include <ros/console.h>
#include <server.h>
#include <sstl/ref_ptr.h>
#include "json.h"
#include "DefaultAgent.h"

using ref_client = sstd::ref_ptr<mysock::socketor>;

struct DispatchServer::Impl : public mysock::Server
{
    using mysock::Server::Server;

    enum class DispatchResult : int
    {
        FAIL = -1,
        DEFAULT_AGENT = 0,
        NEW_AGENT = 1
    };

    static int send_fd(int pipe_fd, SOCKET fd);

    static int recv_fd(int pipe_fd)
    {
        iovec iov[1];
        msghdr msg;
        char buff[0];

        iov[0].iov_base = buff;
        iov[0].iov_len = 1;

        msg.msg_name = nullptr;
        msg.msg_namelen = 0;

        msg.msg_iov = iov;
        msg.msg_iovlen = 1;

        char cmBuf[CMSG_LEN(sizeof(SOCKET))];

        msg.msg_control = cmBuf;
        msg.msg_controllen = CMSG_LEN(sizeof(SOCKET));

        if(recvmsg(pipe_fd, &msg, 0) == -1){
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            ROS_ERROR("recv error, code: ", errCode, " reason: ", reason);
            return 0;
        }

        int fd = *(int *)CMSG_DATA((cmsghdr*)&cmBuf);
        return fd;
    }

    static int dispatch(const ref_client &_client);

    int toDefaultAgent(const ref_client &client);

    int toNewAgent(const ref_client &client);

    DefaultAgent defaultAgent;

};

void DispatchServer::init(uint16_t port)
{
    impl_ = new Impl(port);
    if (impl_->listen() != mysock::LISTEN_SUCESS) {
        ROS_ERROR("Listen failed");
        throw ros::Exception("Listen failed");
    }

    auto pid = fork();

    if (pid < 0)
        throw ros::Exception("fork failed");

    if (pid == 0) {
        impl_->defaultAgent.MAIN();
    } else {
        impl_->defaultAgent.setSocketPipe(pid);
    }

}

void DispatchServer::run()
{
    if (!impl_ || !impl_->isListen()) {
        ROS_ERROR("Server no init or not listen");
        throw ros::Exception("Server no init or not listen");
    }

    ref_client new_agent_cli;
    while (true) {
        auto cli = ref_client(new mysock::Server::Client(impl_->accept()), [](auto p)
            {
                if (auto dp{dynamic_cast<mysock::Server::Client*>(p)}; dp && dp->hasConnected())
                    dp->closeConnect();
                delete p;
            });
        try {
            auto res = (Impl::DispatchResult)impl_->dispatch(cli);
            if (res == Impl::DispatchResult::FAIL)
                ROS_ERROR("Dispatch failed");
            else if (res == Impl::DispatchResult::DEFAULT_AGENT)
                impl_->toDefaultAgent(cli);
            else if (res == Impl::DispatchResult::NEW_AGENT) {
                auto pid = fork();
                if (pid == -1) {
                    auto errCode = errno;
                    auto errMsg = strerror(errCode);
                    ROS_ERROR("Fork failed, error code: %d, error message: %s", errCode, errMsg);
                    throw ros::Exception("Fork failed");
                }
                if (pid == 0) { // sub process
                    new_agent_cli = std::move(cli);
                    break;
                }
            }
        }
        catch (std::exception &e) {
            ROS_ERROR("Catch dispatch exception: %s", e.what());
            ROS_INFO("listen continue");
        }
    }

    if (new_agent_cli) { // sub process
        impl_->toNewAgent(new_agent_cli);
        new_agent_cli = nullptr;
        exit(0);
    }

}
DispatchServer::~DispatchServer()
{
    delete impl_;
}

int DispatchServer::Impl::dispatch(const ref_client &_client)
{
    auto client = dynamic_cast<mysock::Server::Client*>(_client.get());
    char buf[65536];
    if (!client->hasConnected()) {
        ROS_ERROR("Accept error");
        throw ros::Exception("Accept error");
    }
    ROS_INFO("Accept a client, from: %s", client->address().c_str());

    auto len = client->receive(buf, 65536);
    if (len == -1) {
        auto errCode = errno;
        auto errMsg = strerror(errCode);
        ROS_ERROR("Receive error, error code: %d, error message: %s", errCode, errMsg);
        return static_cast<int>(DispatchResult::FAIL);
    }
    if (len == 0) {
        ROS_INFO("connect lost");
        return static_cast<int>(DispatchResult::FAIL);
    }
    buf[len] = 0;

    /*******************************
     * parse param
     ******************************/

    JsonParse json(buf);

    if (json.HasParseError()) {
        auto error_str = "Json parse failed, errorCode: "s + std::to_string(json.GetParseError()) + ", offset: "s
            + std::to_string(json.GetErrorOffset()) + ", error: "s + rapidjson::GetParseError_En(json.GetParseError());
        ROS_ERROR("%s", error_str.c_str());
        client->Send(error_str);
        return static_cast<int>(DispatchResult::FAIL);
    }

    if (!json.HasMember("node") || !json["node"].IsString()) {
        ROS_ERROR("can not found json key: node");
        client->Send("can not found json key: node");
        return static_cast<int>(DispatchResult::FAIL);
    }

    /*******************************
     * parse param over
     ******************************/

    auto node = std::string_view{json["node"].GetString()};

    if (node == "default")
        return static_cast<int>(DispatchResult::DEFAULT_AGENT);
    else
        return static_cast<int>(DispatchResult::NEW_AGENT);

}



int DispatchServer::Impl::toDefaultAgent(const ref_client &client)
{
    ROS_INFO("in %s", __FUNCTION__);
    return static_cast<int>(DispatchResult::DEFAULT_AGENT);
}

int DispatchServer::Impl::toNewAgent(const ref_client &client)
{
    ROS_INFO("in %s", __FUNCTION__);
    closeListen();

    Agent agent;

    agent.MAIN();

    client->Send("create new agent");
    return static_cast<int>(DispatchResult::NEW_AGENT);
}

int DispatchServer::Impl::send_fd(int pipe_fd, SOCKET fd)
{
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
            ROS_ERROR("send error, code: %d, reason: %s", errCode, reason);
            return errCode;
        }
        return 0;
    }
}
