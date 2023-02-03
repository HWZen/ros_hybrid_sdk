//
// Created by HWZen on 2022/12/10.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_GOOGLEPROTOBUFGENERATOR_H
#define ROS_HYBRID_SDK_GOOGLEPROTOBUFGENERATOR_H

#include "../GenCodeResult.h"
#include "../../Parser/typedef.h"

GenCodeResult GenMsgGoogleProtobuf(const std::string &msgFileName, const MsgTrial &vars);

#endif //ROS_HYBRID_SDK_GOOGLEPROTOBUFGENERATOR_H
