//
// Created by HWZen on 2022/11/4.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBIRD_SDK_REFSOCKETOR_H
#define ROS_HYBIRD_SDK_REFSOCKETOR_H
#include "asioHeader.h"
#include "protoData/AgentConfig/AgentConfig.pb.h"

using SOCKET = int;

class Client : public asio::ip::tcp::socket{
public:
    using asio::ip::tcp::socket::socket;
    explicit Client(asio::ip::tcp::socket&& socket) : asio::ip::tcp::socket(std::move(socket)) {}
    hybrid::AgentConfig agentConfig{};
};

using RefSocketor = std::shared_ptr<Client>;

template<typename ...Args>
inline auto make_client(Args &&...args){
    return std::make_shared<Client>(std::forward<Args>(args)...);
}



#endif //ROS_HYBIRD_SDK_REFSOCKETOR_H
