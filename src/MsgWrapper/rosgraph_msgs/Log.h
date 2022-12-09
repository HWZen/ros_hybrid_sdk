
//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on  Fri Dec  9 22:51:42 2022
// Do not Edit!
//
// wrapping message: Log
#pragma once
#include <ros/node_handle.h>
#include "../std_msgs/Header.h"

namespace hybrid {

    struct Log
    {
    static const int8_t DEBUG = 1;
    static const int8_t INFO = 2;
    static const int8_t WARN = 4;
    static const int8_t ERROR = 8;
    static const int8_t FATAL = 16;
    Header header;
    int8_t level;
    std::string name;
    std::string msg;
    std::string file;
    std::string function;
    uint32_t line;
    std::vector<std::string> topics;

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
        LogSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(
                                  Log)> &callback );

        ros::Subscriber sub;
    private:
        ros::NodeHandle nh;
    };

} // namespace hybrid

#ifdef HYBRID_MSG_LIB


template <typename TypeMsg1, typename TypeMsg2>
inline TypeMsg1 LogCover(const TypeMsg2 &Msg2)
{
    TypeMsg1 Msg1;
    Msg1.header = HeaderCover<decltype(Msg1.header), decltype(Msg2.header)>(Msg2.header);
    Msg1.level = Msg2.level;
    Msg1.name = Msg2.name;
    Msg1.msg = Msg2.msg;
    Msg1.file = Msg2.file;
    Msg1.function = Msg2.function;
    Msg1.line = Msg2.line;
    Msg1.topics = Msg2.topics;

    return Msg1;
}

#endif // HYBRID_MSG_LIB
