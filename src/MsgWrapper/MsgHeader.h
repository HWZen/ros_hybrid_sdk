//
// Created by HWZen on 2022/12/7.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_HANDLEWRAPPER_H
#define ROS_HYBRID_SDK_HANDLEWRAPPER_H

#include <cstdint>
#include <string>
#include <ros/time.h>
#include <atomic>

namespace hybrid{
    struct Header{
        uint32_t seq{0};
        ros::Time stamp;
        std::string frame_id;

    };
}

inline hybrid::Header getHeader(const std::string& frame_id = ""){
    static std::atomic_uint32_t seq{0};
    hybrid::Header header;
    header.seq = ++seq;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;
    return header;
}


#endif //ROS_HYBRID_SDK_HANDLEWRAPPER_H
