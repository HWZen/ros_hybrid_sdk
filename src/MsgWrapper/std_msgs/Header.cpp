
//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on  Fri Dec  9 22:51:43 2022
// Do not Edit!
//
// wrapping message: Header
#define HYBRID_MSG_LIB
#include "Header.h"
#include <std_msgs/Header.h>


hybrid::HeaderPublisher::HeaderPublisher(const std::string &topic, uint32_t queue_size, bool latch)
{
    pub = nh.advertise<std_msgs::Header>(topic, queue_size, latch);
}

void hybrid::HeaderPublisher::publish(const hybrid::Header &msg)
{
    pub.publish(HeaderCover<std_msgs::Header,hybrid::Header>(msg));
}


hybrid::HeaderSubscriber::HeaderSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(hybrid::Header)> &callback)
{
    sub = nh.subscribe(topic,queue_size,
                     boost::function<void(
                             const std_msgs::Header::ConstPtr &)>(
                                 [callback](
                                         const std_msgs::Header::ConstPtr &ros_msg)
                                 {
                                     callback(HeaderCover<hybrid::Header,std_msgs::Header>(*ros_msg));
                                 }
                             )
                     );
}
