//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#include "ConnectInstance.h"
#include "../Log.h"
#include "../protoData/Command/Command.pb.h"
#include "../MsgLoader.h"
#include "GlobalVars.h"
#include <utility>
#include <string>
#include <google/protobuf/util/json_util.h>
#include <asio/thread_pool.hpp>

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
    std::unordered_map<std::string, std::shared_ptr<hybrid::SrvCaller>> srvClientMap{};

    awaitable<void> call_server(hybrid::Command command);

    template<asio::completion_token_for<void(bool, std::string)> CompletionToken>
    auto async_call_server(std::shared_ptr<hybrid::SrvCaller> serviceCaller,
                           const std::string &callData,
                           CompletionToken &&token);

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
            client->close();
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

template<asio::completion_token_for<void(bool, std::string)> CompletionToken>
auto ConnectInstance::Impl::async_call_server(std::shared_ptr<hybrid::SrvCaller> serviceCaller,
                                              const std::string &callData,
                                              CompletionToken &&token)
{
    auto init = [this](asio::completion_handler_for<void(bool, std::string)> auto handler,
                       std::shared_ptr<hybrid::SrvCaller> serviceCaller,
                       const std::string &callData)
    {
        auto work = asio::make_work_guard(handler);

        asio::post(*g_serviceCallPool, [
                           work = std::move(work),
                           handler = std::move(handler),
                           callData,
                           serviceCaller = std::move(serviceCaller)
                   ]() mutable
                   {
                       auto alloc = asio::get_associated_allocator(
                               handler, asio::recycling_allocator<void>());
                       auto [res, data] = [&serviceCaller, &callData]()
                       {
                           try {
                               auto resultMsg = serviceCaller->call(callData);
                               return std::make_pair(true, resultMsg);
                           }
                           catch (std::runtime_error &e) {
                               return std::make_pair(false, std::string(e.what()));
                           }
                       }();

                       asio::dispatch(work.get_executor(), asio::bind_allocator(alloc,
                                                                                [
                                                                                        handler = std::move(handler),
                                                                                        res,
                                                                                        data = std::move(data)
                                                                                ]() mutable
                                                                                {
                                                                                    std::move(handler)(res, std::move(data));
                                                                                }
                       ));
                   }
        );
    };

    return asio::async_initiate<CompletionToken, void(bool, std::string)>(init, token, serviceCaller, callData);
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
                                                                              &g_topicQueue,
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
            if (auto it = pubMap.find(publish.topic()); it == pubMap.end()) {
                logger.error("topic {} not found, please advertise it first", publish.topic());
            }
            else{
                try {
                    it->second->publish(publish.data());
                }
                catch (std::runtime_error &e) {
                    logger.error("publish exception: {}", e.what());
                }

            }
            break;
        }
        case hybrid::Command_Type_UNADVERTISE: {
            if (!command.has_unadvertise()) {
                logger.error("unadvertise data not found");
                break;
            }
            const auto &unadvertise = command.unadvertise();
            if (auto it = pubMap.find(unadvertise.topic()); it == pubMap.end()) {
                logger.error("topic {} not found", unadvertise.topic());
                break;
            }
            else
                pubMap.erase(it);
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
                        std::shared_ptr<hybrid::MsgSubscriber>(
                                subscriber_maker(subscribe.topic(),
                                                 subscribe.has_queue_size()
                                                 ? subscribe.queue_size() : 100,
                                                 &g_topicQueue,
                                                 client->agentConfig.is_protobuf(),
                                                 [&, subscribe](const std::string &msg)
                                                 {
                                                     hybrid::Command command;
                                                     command.set_type(hybrid::Command_Type_PUBLISH);
                                                     auto &resPub = *command.mutable_publish();
                                                     resPub.set_topic(subscribe.topic());
                                                     resPub.set_type(subscribe.type());
                                                     resPub.set_data(msg);
                                                     auto resString = command.SerializeAsString() + client->agentConfig.delimiter();
                                                     client->async_write_some(buffer(resString),
                                                                              [&](const asio::error_code &ec, size_t)
                                                                              {
                                                                                  if (ec)
                                                                                      logger.error("send msg error: {}", ec.message());
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
            if (auto it = subMap.find(unsubscribe.topic()); it == subMap.end()) {
                logger.error("topic {} not found", unsubscribe.topic());
                break;
            }
            else
                subMap.erase(it);
            logger.info("unsubscribe topic: {}", unsubscribe.topic());
            break;
        }
        case hybrid::Command_Type_ADVERTISE_SERVICE: {
            logger.error("DefaultAgent not support advertise service, please use signal agent");
            break;
        }
        case hybrid::Command_Type_RESPONSE_SERVICE:{
            logger.error("DefaultAgent not support response service, please use signal agent");
            break;
        }
        case hybrid::Command_Type_CALL_SERVICE:{
            co_spawn(co_await asio::this_coro::executor, call_server(std::move(command)), asio::detached);
            break;
        }
        case hybrid::Command_Type_UNADVERTISE_SERVICE:{
            logger.error("DefaultAgent not support unadvertise service, please use signal agent");
            break;
        }
        case hybrid::Command_Type_LOG:{
            if (!command.has_log()) {
                logger.error("log data not found");
                break;
            }
            const auto &log = command.log();
            this->logger.log(log.level(), log.message());
            break;
        }
        case hybrid::Command_Type_PING:
            break;
        default:
            logger.error("unknown command: {}", command.type());
    }
    co_return;

}

awaitable<void> ConnectInstance::Impl::call_server(hybrid::Command command)
{
    hybrid::Command responseCommand;
    responseCommand.set_type(hybrid::Command_Type_RESPONSE_SERVICE);
    auto &responseService = *responseCommand.mutable_response_service();
    responseService.set_success(false);
    std::string res;
    co_await [&]() -> awaitable<void>{
        if (!command.has_call_service()) {
            this->logger.error("call service data not found");
            responseService.set_error_message("call service data not found");
            co_return;
        }
        const auto &callService = command.call_service();
        auto &serverCaller = srvClientMap[callService.service()];
        if (serverCaller == nullptr) {
            auto client_maker = MsgLoader::getServiceClient(callService.type());
            serverCaller = std::shared_ptr<hybrid::SrvCaller>(client_maker(
                    callService.service(),
                    &g_serviceQueue,
                    client->agentConfig.is_protobuf()));
        }
        auto &callData = client->agentConfig.is_protobuf() ? callService.data() : callService.string_data();
        auto [success, data] = co_await async_call_server(serverCaller, callData, use_nothrow_awaitable);
        if (!success) {
            this->logger.error("call service {} error: {}", callService.service(), data);
            responseService.set_error_message("call service error: "s + data);
        }
        else if (data.empty()) {
            this->logger.error("call service {} failed", callService.service());
            responseService.set_error_message("call service failed");
        }
        else{
            res = data;
            responseService.set_success(true);
        }
        responseService.set_service(callService.service());
        responseService.set_seq(callService.seq());
    }();

    if (client->agentConfig.is_protobuf()){
        responseService.set_data(res);
        co_await client->async_write_some(buffer(responseCommand.SerializeAsString() + client->agentConfig.delimiter()),
                                          use_nothrow_awaitable);
    }
    else {
        responseService.set_string_data(res);
        std::string resBuf;
        auto status = google::protobuf::util::MessageToJsonString(responseCommand, &resBuf);
        if (!status.ok()) {
            this->logger.error("{}@{}: protobuf to json error, reason: {}", __FILE__, __LINE__, status.ToString());
            co_return ;
        }
        co_await client->async_write_some(buffer(resBuf + client->agentConfig.delimiter()),
                                          use_nothrow_awaitable);
    }
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


