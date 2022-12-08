//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_CONFIG_H
#define ROS_HYBRID_SDK_CONFIG_H
#include <vector>
#include <string>

struct Config{
    std::vector<std::string> inputs;
    std::string output;
    bool server{false}, client{false}, onlyServer{false};
};

inline Config g_config;

#endif //ROS_HYBRID_SDK_CONFIG_H
