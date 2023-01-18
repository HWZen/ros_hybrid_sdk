
//
// Created by HWZen on 2022/12/13.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//

#ifndef ROS_HYBRID_DYNAMIC_MSGS_INTERFACE_H
#define ROS_HYBRID_DYNAMIC_MSGS_INTERFACE_H

#include <string>
#include <functional>
#include <ros/callback_queue.h>

namespace hybrid
{
// return ros::Publisher and ros::Subscriber is better?
class MsgPublisher
{
public:
    MsgPublisher(const std::string &topic,
                 uint32_t queue_size,
                 ros::CallbackQueue* callbackQueue,
                 bool is_protobuf,
                 bool latch = false) {};

    virtual void publish(const std::string &msg) = 0;

    virtual ~MsgPublisher() = default;

};

class MsgSubscriber
{
public:
    MsgSubscriber(const std::string &topic,
                  uint32_t queue_size,
                  ros::CallbackQueue* callbackQueue,
                  bool is_protobuf,
                  const std::function<void(std::string)> &callback) {};

    virtual ~MsgSubscriber() = default;

};

class SrvAdvertiser
{
public:
    SrvAdvertiser(const std::string &topic,
                  ros::CallbackQueue* callbackQueue,
                  bool is_protobuf,
                  const std::function<std::string(std::string)> &callback) {};
    uint64_t seq{0};
    virtual ~SrvAdvertiser() = default;

};

class SrvCaller
{
public:
    SrvCaller(const std::string &topic, ros::CallbackQueue* callbackQueue, bool is_protobuf) {};

    virtual std::string call(const std::string &req) = 0;

    virtual ~SrvCaller() = default;

    uint64_t seq{0};

    std::unordered_map<uint64_t,void *> reqMap{};

};

}

#endif //ROS_HYBRID_DYNAMIC_MSGS_INTERFACE_H
