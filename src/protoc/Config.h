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
    bool server{false};
    bool client{false};
    bool onlyServer{false};
    std::string packagePath;
    bool genProtobuf{false};
    bool genServerCode{false};
    bool genCmake{false};
    bool buildProtobuf{false};
    bool buildServerMsg{false};
    std::string protocPath{"protoc"};

};

inline Config g_config;

#endif //ROS_HYBRID_SDK_CONFIG_H
