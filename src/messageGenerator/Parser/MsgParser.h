//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_MSGPARSER_H
#define ROS_HYBRID_SDK_MSGPARSER_H

#include "Preprocessing.h"
#include "typedef.h"
#include <regex>




inline std::vector<TypeTrail> MsgParser(const std::string& fileBuf){

    auto preprocessRes = Preprocessing(fileBuf);

    std::vector<TypeTrail> res;
    for (auto& var : preprocessRes){
        res.emplace_back(TypeTrailParser(var));
    }

    return res;

}

#endif //ROS_HYBRID_SDK_MSGPARSER_H
