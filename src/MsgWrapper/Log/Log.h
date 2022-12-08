//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on Dec  8 2022  22:00:32
// Do not Edit!
//
// wrapping message: Log

#pragma once
#include <ros/node_handle.h>
#include "../MsgHeader.h"
#include <vector>

namespace hybrid {

    struct Log
    {
        static const int8_t DEBUG = 1;
        static const int8_t INFO = 2;
        static const int8_t WARN = 4;
        static const int8_t ERROR = 8;
        static const int8_t FATAL = 16;



        Header header{};
        int8_t level{};
        std::string name{};
        std::string msg{};
        std::string file{};
        std::string function{};
        uint32_t line{};
        std::vector<std::string> topics{};

        };

    class LogPublisher{
    public:
        LogPublisher(const std::string &topic, uint32_t queue_size, bool latch = false);
        void publish(const Log &msg);

        ros::Publisher pub;
    private:
        ros::NodeHandle nh;
    };

    class LogSubscriber{
    public:
        LogSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(Log)> &callback );

        ros::Subscriber sub;
    private:
        ros::NodeHandle nh;
    };

}
