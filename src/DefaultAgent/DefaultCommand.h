//
// Created by HWZen on 2022/12/11.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_DEFAULTCOMMAND_H
#define ROS_HYBRID_SDK_DEFAULTCOMMAND_H

#include "../CommandMsg/Command.pb.h"
#include "../Log.h"
#include <ros/ros.h>

namespace hybrid{
    class MsgPublisher{
    public:
        MsgPublisher(const std::string &topic, uint32_t queue_size, bool latch = false){};
        virtual void publish(const std::string &msg) = 0;
        virtual ~MsgPublisher() = default;

    protected:
        ros::NodeHandle nh{};
        ros::Publisher pub{};
    };

    class MsgSubscriber{
    public:
        MsgSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(std::string)>& callback){};
        virtual ~MsgSubscriber() = default;
    };

}

class DefaultCommand
{
public:
    std::string test(const std::string &buffer);
    auto getPublisher(const std::string& name);
    auto getSubscriber(const std::string& name);
private:
    Log logger{"DefaultCommand", LogFlag::CONSOLE_LOGGER | LogFlag::ROS_LOGGER};

    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgPublisher>> pubMap{};

    std::unordered_map<std::string, hybrid::MsgPublisher *(*)(const std::string &, uint32_t, bool)> pubFuncMap{};
};



#endif //ROS_HYBRID_SDK_DEFAULTCOMMAND_H
