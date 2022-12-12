//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_GENMSGSERVERUSEONLY_H
#define ROS_HYBRID_SDK_GENMSGSERVERUSEONLY_H

#include "GenCodeResult.h"
#include "../Parser/typedef.h"

GenCodeResult GenMsgServerUseOnly(const std::string &msgFileName, const std::vector<TypeTrail> &vars);

#endif //ROS_HYBRID_SDK_GENMSGSERVERUSEONLY_H
