//
// Created by HWZ on 2022/11/2.
//

#include "DispatchServer.h"
#include <ros/console.h>
#include <server.h>
#include <sstl/ref_ptr.h>
#include "json/json.h"

struct DispatchServer::DispatcherImpl : public mysock::Server
{
    using mysock::Server::Server;

    enum class DispatchResult: int
    {
        FAIL                = -1,
        DEFAULT_AGENT       = 0,
        NEW_AGENT           = 1,
        MAIN_SERVER         = 2,
    } ;

    int dispatch(const sstd::ref_ptr<Client> &_client);

    int toDefaultAgent(const sstd::ref_ptr<Client> &client);

    int toNewAgent(const sstd::ref_ptr<Client> &client);

};

void DispatchServer::init(uint16_t port)
{
    impl_ = new DispatcherImpl(port);
    if (impl_->listen() != mysock::LISTEN_SUCESS) {
        ROS_ERROR("Listen failed");
        throw ros::Exception("Listen failed");
    }

}

void DispatchServer::run()
{
    if (!impl_ || !impl_->isListen()) {
        ROS_ERROR("Server no init or not listen");
        throw ros::Exception("Server no init or not listen");
    }

    sstd::ref_ptr<mysock::Server::Client> new_agent_cli;
    while (true) {
        auto cli = sstd::ref_ptr(new mysock::Server::Client(impl_->accept()), [](auto p)
        {
            if (p){
                p->closeConnect();
            }
             delete p;
        });
        try {
            auto res = (DispatcherImpl::DispatchResult)impl_->dispatch(cli);
            if (res == DispatcherImpl::DispatchResult::FAIL)
                ROS_ERROR("Dispatch failed");
            if(res == DispatcherImpl::DispatchResult::DEFAULT_AGENT)
                impl_->toDefaultAgent(cli);
            else if(res == DispatcherImpl::DispatchResult::NEW_AGENT){
                auto pid = fork();
                if (pid == -1){
                    auto errCode = errno;
                    auto errMsg = strerror(errCode);
                    ROS_ERROR("Fork failed, error code: %d, error message: %s", errCode, errMsg);
                    throw ros::Exception("Fork failed");
                }
                if(pid == 0){ // sub process
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

    if (new_agent_cli){ // sub process
        impl_->toNewAgent(new_agent_cli);
        new_agent_cli = nullptr;
        exit(0);
    }

}
DispatchServer::~DispatchServer()
{
    delete impl_;
}

int DispatchServer::DispatcherImpl::dispatch(const sstd::ref_ptr<Client> &_client)
{
    auto client = std::move(_client);
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

    /*******************************
     * parse param
     ******************************/

    JsonParser json(buf);
    if (json.empty()) {
        ROS_ERROR("Json parse failed, or json is empty");
        client->Send("Json parse failed, or json is empty");
        return static_cast<int>(DispatchResult::FAIL);
    }

    if (!json.isMember("node") || !json["node"].isString()) {
        ROS_ERROR("can not found json key: communicationType or node");
        client->Send("can not found json key: communicationType or node");
        return static_cast<int>(DispatchResult::FAIL);
    }

    /*******************************
     * parse param over
     ******************************/

    auto node = json["node"].asString();

    if (node == "default")
        return static_cast<int>(DispatchResult::DEFAULT_AGENT);
    else
        return static_cast<int>(DispatchResult::NEW_AGENT);

}
int DispatchServer::DispatcherImpl::toDefaultAgent(const sstd::ref_ptr<Client> &client)
{
    ROS_INFO("in %s", __FUNCTION__);
    return static_cast<int>(DispatchResult::DEFAULT_AGENT);
}
int DispatchServer::DispatcherImpl::toNewAgent(const sstd::ref_ptr<Client> &client)
{
    ROS_INFO("in %s", __FUNCTION__);

    closeListen();

    client->Send("create new agent");
    return static_cast<int>(DispatchResult::NEW_AGENT);
}
