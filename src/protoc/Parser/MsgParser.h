//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_MSGPARSER_H
#define ROS_HYBRID_SDK_MSGPARSER_H

#include "typedef.h"
#include <vector>

std::vector<TypeTrail> MsgParser(const std::string &fileBuf);

#endif //ROS_HYBRID_SDK_MSGPARSER_H
