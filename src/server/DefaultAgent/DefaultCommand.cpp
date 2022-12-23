//
// Created by HWZen on 2022/12/11.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#include "DefaultCommand.h"
#include "../protoData/Command/Command.pb.h"
#include "../MsgLoader.h"

struct Test{

};


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

            const auto &advertise = command.advertise();

            try{
                auto publisher_maker = MsgLoader::getPublisher(command.advertise().type());
                pubMap[advertise.topic()] = std::shared_ptr<hybrid::MsgPublisher>{publisher_maker(advertise.topic(), advertise.queue_size(), advertise.latch())};
                res = command;
            }
            catch (std::runtime_error &e){
                logger.error("publish exception: {}", e.what());
                res.mutable_log()->set_message(e.what());
                break;
            }
            break;
        }
        case hybrid::Command_Type_PUBLISH:
        {
            if (!command.has_publish()){
                logger.error("Command has no publish");
                res.mutable_log()->set_message("Command has no publish");
                break;
            }
            const auto &publish = command.publish();
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