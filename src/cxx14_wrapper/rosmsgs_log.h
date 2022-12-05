//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_ROSMSGS_LOG_H
#define ROS_HYBRID_SDK_ROSMSGS_LOG_H

#include <string>

struct RosLogWrapper
{
    char level;
    std::string name;
    std::string msg;
};

class RosLogPublisher{
public:
    RosLogPublisher();

    void publish(const RosLogWrapper &log);

private:
    struct Impl;
    Impl *implPtr;
};

#endif //ROS_HYBRID_SDK_ROSMSGS_LOG_H
