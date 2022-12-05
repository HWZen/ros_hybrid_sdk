//
// Created by HWZen on 2022/12/5.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#include "ConnectInstance.h"
#include "../Log.h"
#include <utility>
#include <string>

using namespace std::string_literals;

struct ConnectInstance::Impl
{

    explicit Impl(RefSocketor client) : client(std::move(client)) {}

    RefSocketor client;

    int MAIN();

    Log logger{"connect"s + std::to_string(client->native_handle()),
               static_cast<LogFlag>(LogFlag::CONSOLE_CLIENT | LogFlag::ROS_LOGGER), client};

};

int ConnectInstance::Impl::MAIN()
{
    logger.info("received: {}:{}, next operation not implement", client->remote_endpoint().address().to_string(),
                client->remote_endpoint().port());
    return 0;
}

ConnectInstance::ConnectInstance(RefSocketor client)
{
    Log tmpLog{"temp"};
    implPtr = new Impl(std::move(client));

}

int ConnectInstance::MAIN()
{
    return implPtr->MAIN();
}

ConnectInstance::~ConnectInstance()
{
    delete implPtr;
}


