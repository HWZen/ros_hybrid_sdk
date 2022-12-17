//
// Created by HWZen on 2022/12/11.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#include "DefaultCommand.h"
#include "../CommandMsg/Command.pb.h"
#include <dlfcn.h>
#include <functional>

struct Test{

};





auto DefaultCommand::getPublisher(const std::string& name){

    if (pubFuncMap.count(name))
        return pubFuncMap[name];
    auto dllName = "lib" + name + ".so";
    auto res = dlopen(dllName.c_str(), RTLD_LAZY);
    if (!res){
        logger.error("dlopen error: {}", dlerror());
        return (hybrid::MsgPublisher *(*)(const std::string &, uint32_t, bool)) nullptr;
    }
    auto pVoid = dlsym(res, "make_publisher");
    if (!pVoid){
        logger.error("dlsym error: {}", dlerror());
        return (hybrid::MsgPublisher *(*)(const std::string &, uint32_t, bool)) nullptr;
    }
    auto func = reinterpret_cast<hybrid::MsgPublisher *(*)(const std::string &, uint32_t, bool)>(pVoid);
    pubFuncMap[name] = func;
    return func;
}

auto DefaultCommand::getSubscriber(const std::string& name){
    static std::unordered_map<std::string, hybrid::MsgSubscriber *(*)(const std::string &, uint32_t, const std::function<void(std::string)>&)> subFuncMap{};
    if (subFuncMap.count(name))
        return subFuncMap[name];
    auto res = dlopen(name.c_str(), RTLD_LAZY);
    if (!res){
        logger.error("dlopen error: %s", dlerror());
        return (hybrid::MsgSubscriber *(*)(const std::string &, uint32_t, const std::function<void(std::string)>&)) nullptr;
    }
    auto pVoid = dlsym(res, "make_subscriber");
    if (!pVoid){
        logger.error("dlsym error: %s", dlerror());
        return (hybrid::MsgSubscriber *(*)(const std::string &, uint32_t, const std::function<void(std::string)>&)) nullptr;
    }
    auto func = reinterpret_cast<hybrid::MsgSubscriber *(*)(const std::string &, uint32_t, const std::function<void(std::string)>&)>(pVoid);
    subFuncMap[name] = func;
    return func;
}


std::string DefaultCommand::test(const std::string &buffer)
{
    hybrid::Command command;
    if (!command.ParseFromString(buffer))
        throw std::runtime_error("parse command error");

//    logger.debug("recv Command: {}", command.DebugString());
    hybrid::Command res{};
    res.set_type(hybrid::Command_Type_LOG);
    res.mutable_log()->set_level(hybrid::Command_Log_Level_ERROR);
    res.mutable_log()->set_message("no set reason, may be no implement");
    switch(command.type()){
        case hybrid::Command_Type_UNKNOWN:
            res.mutable_log()->set_message("unknown command");
            break;
        case hybrid::Command_Type_ADVERTISE:
        {
            if (!command.has_advertise()){
                logger.error("Command has no advertise");
                res.mutable_log()->set_message("Command has no advertise");
                break;
            }

            auto &advertise = command.advertise();

            auto publisher_maker = getPublisher(command.advertise().type());
            if(!publisher_maker){
                logger.error("get publisher maker error");
                res.mutable_log()->set_message("get publisher maker error");
                break;
            }
            pubMap[advertise.topic()] = std::shared_ptr<hybrid::MsgPublisher>{publisher_maker(advertise.topic(), advertise.queue_size(), advertise.latch())};
            res = command;
            break;
        }
        case hybrid::Command_Type_PUBLISH:
        {
            if (!command.has_publish()){
                logger.error("Command has no publish");
                res.mutable_log()->set_message("Command has no publish");
                break;
            }
            auto &publish = command.publish();
            if (!pubMap.count(publish.topic())){
                logger.error("topic not found");
                res.mutable_log()->set_message("topic not found");
                break;
            }
            pubMap[publish.topic()]->publish(publish.data());
            res = command;
            break;
        }
        case hybrid::Command_Type_UNADVERTISE:
        {
            if (!command.has_unadvertise()){
                logger.error("Command has no unadvertise");
                res.mutable_log()->set_message("Command has no unadvertise");
                break;
            }
            auto &unadvertise = command.unadvertise();
            if (!pubMap.count(unadvertise.topic())){
                logger.error("topic not found");
                res.mutable_log()->set_message("topic not found");
                break;
            }
            pubMap.erase(unadvertise.topic());
            res = command;
            break;
        }
        case hybrid::Command_Type_SUBSCRIBE:
            break;
        case hybrid::Command_Type_UNSUBSCRIBE:
            break;
        case hybrid::Command_Type_ADVERTISE_SERVICE:
            break;
        case hybrid::Command_Type_CALL_SERVICE:
            break;
        case hybrid::Command_Type_RESPONSE_SERVICE:
            break;
        case hybrid::Command_Type_UNADVERTISE_SERVICE:
            break;
        case hybrid::Command_Type_PING:
            res.set_type(hybrid::Command_Type_PING);
            break;
        default:
            break;
    }
    return res.SerializeAsString();
}