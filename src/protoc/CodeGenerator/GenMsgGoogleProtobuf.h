//
// Created by HWZen on 2022/12/10.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_GOOGLEPROTOBUFGENERATOR_H
#define ROS_HYBRID_SDK_GOOGLEPROTOBUFGENERATOR_H

#include "GenMsgCodeResult.h"
#include "../Parser/typedef.h"

GenCodeResult GenGoogleProtobuf(const std::string &msgFileName, const std::vector<TypeTrail> &vars);

#endif //ROS_HYBRID_SDK_GOOGLEPROTOBUFGENERATOR_H
