//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#include "ConnectInstance.h"
#include "../Log.h"
#include "DefaultCommand.h"
#include <utility>
#include <string>
#include "../asioHeader.h"
#include "spdlog/fmt/bin_to_hex.h"

using namespace std::string_literals;

struct ConnectInstance::Impl
{

    explicit Impl(RefSocketor client) : client(std::move(client)) {}

    RefSocketor client;

    awaitable<int> MAIN();

    Log logger{"connect"s + std::to_string(client->native_handle()),
               static_cast<LogFlag>(LogFlag::CONSOLE_CLIENT | LogFlag::ROS_LOGGER), client};

};


awaitable<int> ConnectInstance::Impl::MAIN() try
{
//    logger.info("received: {}:{}, next operation not implement", client->remote_endpoint().address().to_string(),
//                client->remote_endpoint().port());

    std::string read_buffer;
    DefaultCommand cmd;
    for (;;){

        auto delimiter = HYBRID_DELIMITER;
        auto [ec, len] = co_await asio::async_read_until(*client, asio::dynamic_buffer(read_buffer, 4096), delimiter, use_nothrow_awaitable);

        if (ec && ec != asio::error::eof) {
            logger.error("read error: {}", ec.message());
            co_return -1;
        }
        if (ec == asio::error::eof) {
            logger.info("disconnect");
            co_return 0;
        }

        auto res = cmd.test(read_buffer.substr(0, len - HYBRID_DELIMITER_SIZE));
        auto [ec2, len2] = co_await client->async_write_some(buffer(res), use_nothrow_awaitable);
        if (ec2 && ec2 != asio::error::eof) {
            logger.error("write error: {}", ec2.message());
            co_return -1;
        }
        else if (ec2 == asio::error::eof) {
            logger.info("disconnect");
            co_return 0;
        }
        read_buffer.erase(0, len);
    }

    co_return 0;
}
catch(std::exception &e) {
    logger.error("catch exception: {}", e.what());
    co_return 1;
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


