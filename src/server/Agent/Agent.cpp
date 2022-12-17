//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "../Log.h"
#include "../../SDKException.h"
#include "../asioHeader.h"
#include "../CommandMsg/Command.pb.h"
using namespace std::string_literals;

struct Agent::Impl
{
    Impl(const RefSocketor &client, std::string agentName);

    awaitable<void> MAIN();

    std::string agentName;

    Log logger;

    RefSocketor client;

    awaitable<void> parseCommand(const std::string &commandStr);
};


Agent::Impl::Impl(const RefSocketor &client, std::string agentName) : logger("Agent_"s + Impl::agentName,
                                                                             LogFlag::CONSOLE_CLIENT,
                                                                             client),
                                                                    agentName(std::move(agentName)),
                                                                    client(client)
{
}

awaitable<void> Agent::MAIN()
{
    if (!implPtr) [[unlikely]]
        throw SDKException("Agent::MAIN() implPtr is nullptr");
    co_return co_await implPtr->MAIN();
}


awaitable<void> Agent::Impl::MAIN()
{
    try
    {
        std::string read_buffer;
        for (;;){
            auto [ec, len] = co_await asio::async_read_until(*client, asio::dynamic_buffer(read_buffer, 1024 * 8), HYBRID_DELIMITER, use_nothrow_awaitable);
            if (ec) [[unlikely]]
            {
                logger.error("read error: {}", ec.message());
                logger.info("exit");
                co_return;
            }
            read_buffer.resize(len - HYBRID_DELIMITER_SIZE);
            parseCommand(read_buffer);
        }
    }
    catch (const std::exception &e)
    {
        logger.error("catch exception: ", e.what());
    }
}

Agent::Agent(const RefSocketor &client, const std::string &agentName) : implPtr(new Impl(client, agentName))
{
}

Agent::~Agent()
{
    delete implPtr;
}

awaitable<void> Agent::Impl::parseCommand(const std::string &commandStr)
{
    hybrid::Command command;
    if (!command.ParseFromString(commandStr))
        logger.error("parse command error");
//    logger.debug("command: {}", command.DebugString());

    switch (command.type()) {
    case hybrid::Command_Type_UNKNOWN:break;
    case hybrid::Command_Type_ADVERTISE:
    {
        if (!command.has_advertise()){
            logger.error("advertise data not found");
            break;
        }
//        auto publisher_maker = getPublisher(command.advertise().type());
        break;
    }
    case hybrid::Command_Type_PUBLISH:break;
    case hybrid::Command_Type_UNADVERTISE:break;
    case hybrid::Command_Type_SUBSCRIBE:break;
    case hybrid::Command_Type_UNSUBSCRIBE:break;
    case hybrid::Command_Type_ADVERTISE_SERVICE:break;
    case hybrid::Command_Type_CALL_SERVICE:break;
    case hybrid::Command_Type_RESPONSE_SERVICE:break;
    case hybrid::Command_Type_UNADVERTISE_SERVICE:break;
    case hybrid::Command_Type_LOG:break;
    case hybrid::Command_Type_PING:break;
    case hybrid::Command_Type_Command_Type_INT_MIN_SENTINEL_DO_NOT_USE_:break;
    case hybrid::Command_Type_Command_Type_INT_MAX_SENTINEL_DO_NOT_USE_:break;
    }

}
