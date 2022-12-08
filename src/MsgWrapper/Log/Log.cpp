//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on Dec  8 2022  22:00:32
// Do not Edit!
//
// wrapping message: Log

#include "Log.h"
#include <rosgraph_msgs/Log.h>


hybrid::LogPublisher::LogPublisher(const std::string &topic, uint32_t queue_size, bool latch)
{
    pub = nh.advertise<rosgraph_msgs::Log>(topic, queue_size, latch);
}


void hybrid::LogPublisher::publish(const hybrid::Log &msg)
{
    rosgraph_msgs::Log ros_msg;

    ros_msg.header.stamp = msg.header.stamp;
    ros_msg.header.seq = msg.header.seq;
    ros_msg.header.frame_id = msg.header.frame_id;
    ros_msg.level = msg.level;
    ros_msg.name = msg.name;
    ros_msg.msg = msg.msg;
    ros_msg.file = msg.file;
    ros_msg.function = msg.function;
    ros_msg.line = msg.line;
    ros_msg.topics = msg.topics;

    pub.publish(ros_msg);
}

hybrid::LogSubscriber::LogSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(hybrid::Log)> &callback )
{
    sub = nh.subscribe(topic,queue_size,
                     boost::function<void(
                             const rosgraph_msgs::Log::ConstPtr &)>(
                             [callback](
                                     const rosgraph_msgs::Log::ConstPtr &ros_msg)
                             {
                                hybrid::Log msg;

                                msg.header.stamp = ros_msg->header.stamp;
                                msg.header.seq = ros_msg->header.seq;
                                msg.header.frame_id = ros_msg->header.frame_id;
                                msg.level = ros_msg->level;
                                msg.name = ros_msg->name;
                                msg.msg = ros_msg->msg;
                                msg.file = ros_msg->file;
                                msg.function = ros_msg->function;
                                msg.line = ros_msg->line;
                                msg.topics = ros_msg->topics;

                                callback(std::move(msg));
                             }));
}

