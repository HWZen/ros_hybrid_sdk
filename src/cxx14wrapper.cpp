//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//

#include "cxx14_wrapper/rosmsgs_log.h"

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

struct RosLogPublisher::Impl{
    ros::NodeHandle nh{};
    ros::Publisher pub = nh.advertise<rosgraph_msgs::Log>("/rosout", 1000);
};

RosLogPublisher::RosLogPublisher()
{
    implPtr = new Impl;
}

void RosLogPublisher::publish(const RosLogWrapper &log)
{
    rosgraph_msgs::Log msg;
    msg.level = log.level;
    msg.name = log.name;
    msg.msg = log.msg;
    msg.header.stamp = ros::Time::now();
    static std::atomic_uint32_t seq{0};
    msg.header.seq = ++seq;
    implPtr->pub.publish(msg);
}

