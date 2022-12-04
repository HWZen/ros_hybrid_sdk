//
// Created by HWZen on 2022/12/4.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBIRD_SDK_SDKEXCEPTION_H
#define ROS_HYBIRD_SDK_SDKEXCEPTION_H
#include <stdexcept>

class SDKException : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};


#endif //ROS_HYBIRD_SDK_SDKEXCEPTION_H
