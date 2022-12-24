//
// Created by HWZen on 2022/12/11.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_DEFAULTCOMMAND_H
#define ROS_HYBRID_SDK_DEFAULTCOMMAND_H

#include "../Log.h"
#include "../Interface.h"

class DefaultCommand
{
public:
    std::string test(const std::string &buffer);
private:
    Log logger{"DefaultCommand", LogFlag::CONSOLE_LOGGER | LogFlag::ROS_LOGGER};

    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgPublisher>> pubMap{};

    std::unordered_map<std::string, hybrid::MsgPublisher *(*)(const std::string &, uint32_t, bool)> pubFuncMap{};
};

#endif //ROS_HYBRID_SDK_DEFAULTCOMMAND_H
