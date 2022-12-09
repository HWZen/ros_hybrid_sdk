
//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on  Fri Dec  9 22:51:42 2022
// Do not Edit!
//
// wrapping message: Log
#define HYBRID_MSG_LIB
#include "Log.h"
#include <rosgraph_msgs/Log.h>


hybrid::LogPublisher::LogPublisher(const std::string &topic, uint32_t queue_size, bool latch)
{
    pub = nh.advertise<rosgraph_msgs::Log>(topic, queue_size, latch);
}

void hybrid::LogPublisher::publish(const hybrid::Log &msg)
{
    pub.publish(LogCover<rosgraph_msgs::Log,hybrid::Log>(msg));
}


hybrid::LogSubscriber::LogSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(hybrid::Log)> &callback)
{
    sub = nh.subscribe(topic,queue_size,
                     boost::function<void(
                             const rosgraph_msgs::Log::ConstPtr &)>(
                                 [callback](
                                         const rosgraph_msgs::Log::ConstPtr &ros_msg)
                                 {
                                     callback(LogCover<hybrid::Log,rosgraph_msgs::Log>(*ros_msg));
                                 }
                             )
                     );
}
