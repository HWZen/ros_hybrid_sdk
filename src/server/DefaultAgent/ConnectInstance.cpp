//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#include "ConnectInstance.h"
#include "../Log.h"
#include "../protoData/Command/Command.pb.h"
#include "../MsgLoader.h"
#include "CallbackQueues.h"
#include <utility>
#include <string>
#include <google/protobuf/util/json_util.h>

using namespace std::string_literals;

struct ConnectInstance::Impl
{

    explicit Impl(RefSocketor client) : client(std::move(client)) {}

    RefSocketor client;

    awaitable<int> MAIN();

    Log logger{"connect"s + std::to_string(client->native_handle()),
               LogFlag::CONSOLE_CLIENT, client};

    awaitable<void> parseCommand(std::string_view commandStr);

    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgPublisher>> pubMap{};
    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgSubscriber>> subMap{};

};

awaitable<int> ConnectInstance::Impl::MAIN() try
{
    logger.info("login success");
    std::string read_buffer;
    for (;;) {

        auto [ec, len] = co_await asio::async_read_until(*client,
                                                         asio::dynamic_buffer(read_buffer, 1024 * 8),
                                                         client->agentConfig.delimiter(),
                                                         use_nothrow_awaitable);

        if (ec && ec != asio::error::eof) [[unlikely]] {
            logger.error("read error: {}", ec.message());
            co_return -1;
        }
        if (ec == asio::error::eof) {
            logger.info("disconnect");
            co_return 0;
        }

        co_await parseCommand(std::string_view(read_buffer.data(), len - client->agentConfig.delimiter().size()));
        read_buffer.erase(0, len);
    }

    co_return 0;
}
catch (std::exception &e) {
    logger.error("catch exception: {}", e.what());
    co_return 1;
}
awaitable<void> ConnectInstance::Impl::parseCommand(std::string_view commandStr)
{
    hybrid::Command command;

    if (client->agentConfig.is_protobuf()) {
        if (!command.ParseFromArray(commandStr.data(), static_cast<int>(commandStr.size())))
            logger.error("parse command error");
    } else {
        if (auto state = google::protobuf::util::JsonStringToMessage(commandStr, &command); !state.ok())
            logger.error("parse command error: {}", state.ToString());
    }
//    logger.debug("command: {}", command.DebugString());

    switch (command.type()) {
    case hybrid::Command_Type_UNKNOWN:
        break;
    case hybrid::Command_Type_ADVERTISE: {
        if (!command.has_advertise()) {
            logger.error("advertise data not found");
            break;
        }
        const auto &advertise = command.advertise();
        if (pubMap.count(advertise.topic()) > 0) {
            logger.error("topic {} already exist", advertise.topic());
            break;
        }

        try {
            auto publisher_maker = MsgLoader::getPublisher(advertise.type());
            pubMap[advertise.topic()] =
                std::shared_ptr<hybrid::MsgPublisher>(publisher_maker(advertise.topic(),
                                                                      advertise.has_queue_size()
                                                                      ? advertise.queue_size() : 100,
                                                                      &topicQueue,
                                                                      client->agentConfig.is_protobuf(),
                                                                      advertise.has_latch() && advertise.latch()));
            logger.info("advertise topic: {}", advertise.topic());
        }
        catch (std::runtime_error &e) {
            logger.error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_PUBLISH: {
        if (!command.has_publish()) {
            logger.error("publish data not found");
            break;
        }
        if (command.mutable_publish()->has_string_data())
            command.mutable_publish()->set_data(command.mutable_publish()->string_data());
        const auto &publish = command.publish();
        if (pubMap.count(publish.topic()) == 0) {
            logger.error("topic {} not found, please advertise it first", publish.topic());
            break;
        }
        try {
            pubMap[publish.topic()]->publish(publish.data());
        }
        catch (std::runtime_error &e) {
            logger.error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNADVERTISE: {
        if (!command.has_unadvertise()) {
            logger.error("unadvertise data not found");
            break;
        }
        const auto &unadvertise = command.unadvertise();
        if (pubMap.count(unadvertise.topic()) == 0) {
            logger.error("topic {} not found", unadvertise.topic());
            break;
        }
        pubMap.erase(unadvertise.topic());
        logger.info("unadvertise topic: {}", unadvertise.topic());
        break;
    }
    case hybrid::Command_Type_SUBSCRIBE: {
        if (!command.has_subscribe()) {
            logger.error("subscribe data not found");
            break;
        }
        const auto &subscribe = command.subscribe();
        if (subMap.count(subscribe.topic()) > 0) {
            logger.error("topic {} already exist", subscribe.topic());
            break;
        }
        try {
            logger.info("subscribe topic: {}", subscribe.topic());
            auto subscriber_maker = MsgLoader::getSubscriber(subscribe.type());
            subMap[subscribe.topic()] =
                std::shared_ptr<hybrid::MsgSubscriber>(subscriber_maker(subscribe.topic(),
                                                                        subscribe.has_queue_size()
                                                                        ? subscribe.queue_size() : 100,
                                                                        &topicQueue,
                                                                        client->agentConfig.is_protobuf(),
                                                                        [&, subscribe](const std::string &msg)
                                                                        {
                                                                            hybrid::Command command;
                                                                            command.set_type(hybrid::Command_Type_PUBLISH);
                                                                            auto &resPub = *command.mutable_publish();
                                                                            resPub.set_topic(subscribe.topic());
                                                                            resPub.set_type(subscribe.type());
                                                                            resPub.set_data(msg);
                                                                            auto resString = command.SerializeAsString()
                                                                                + client->agentConfig.delimiter();
                                                                            client->async_write_some(buffer(resString),
                                                                                                     [&](const asio::error_code &ec,
                                                                                                         size_t)
                                                                                                     {
                                                                                                         if (ec)
                                                                                                             logger.error(
                                                                                                                 "send msg error: {}",
                                                                                                                 ec.message());
                                                                                                     });
                                                                        }
                ));
        }
        catch (std::runtime_error &e) {
            logger.error("subscribe exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNSUBSCRIBE: {
        if (!command.has_unsubscribe()) {
            logger.error("unsubscribe data not found");
            break;
        }
        const auto &unsubscribe = command.unsubscribe();
        if (subMap.count(unsubscribe.topic()) == 0) {
            logger.error("topic {} not found", unsubscribe.topic());
            break;
        }
        subMap.erase(unsubscribe.topic());
        logger.info("unsubscribe topic: {}", unsubscribe.topic());
        break;
    }
    case hybrid::Command_Type_ADVERTISE_SERVICE:
    case hybrid::Command_Type_CALL_SERVICE:
    case hybrid::Command_Type_RESPONSE_SERVICE:
    case hybrid::Command_Type_UNADVERTISE_SERVICE:
    case hybrid::Command_Type_LOG:
    case hybrid::Command_Type_PING:
    case hybrid::Command_Type_Command_Type_INT_MIN_SENTINEL_DO_NOT_USE_:
    case hybrid::Command_Type_Command_Type_INT_MAX_SENTINEL_DO_NOT_USE_:
        logger.info("command not implement");
        break;
    default:
        logger.error("unknown command: {}", command.type());
    }
    co_return;
}

ConnectInstance::ConnectInstance(RefSocketor client)
{
    Log tmpLog{"temp"};
    implPtr = new Impl(std::move(client));
}

awaitable<int> ConnectInstance::MAIN()
{
    co_return co_await implPtr->MAIN();
}

ConnectInstance::~ConnectInstance()
{
    delete implPtr;
}


