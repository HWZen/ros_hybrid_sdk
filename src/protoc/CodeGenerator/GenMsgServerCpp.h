//
// Created by HWZen on 2022/12/12.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_GENMSGSERVER_H
#define ROS_HYBRID_SDK_GENMSGSERVER_H

#include "GenMsgCodeResult.h"
#include "../Parser/typedef.h"

GenCodeResult GenMsgServerCpp(const std::string &msgFileName, const std::vector<TypeTrail> &vars);

#endif //ROS_HYBRID_SDK_GENMSGSERVER_H
