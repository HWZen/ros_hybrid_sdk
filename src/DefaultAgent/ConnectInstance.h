//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBIRD_SDK_CONNECTINSTANCE_H
#define ROS_HYBIRD_SDK_CONNECTINSTANCE_H

#include "../RefSocketor.h"
#include <memory>
#include <asio/experimental/awaitable_operators.hpp>

class ConnectInstance
{
public:
    ConnectInstance(RefSocketor client);

    ~ConnectInstance();

    asio::awaitable<int> MAIN();
private:
    struct Impl;
    Impl *implPtr;
};

using RefConnectInstance = std::shared_ptr<ConnectInstance>;

#endif //ROS_HYBIRD_SDK_CONNECTINSTANCE_H
