//
// Created by HWZen on 2022/11/4.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBIRD_SDK_REFSOCKETOR_H
#define ROS_HYBIRD_SDK_REFSOCKETOR_H
#include <asio/ip/tcp.hpp>

using SOCKET = int;
using RefSocketor = std::shared_ptr<asio::ip::tcp::socket>;

#endif //ROS_HYBIRD_SDK_REFSOCKETOR_H
