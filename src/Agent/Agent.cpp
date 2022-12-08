//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "../Log.h"
#include "../SDKException.h"
using namespace std::string_literals;

struct Agent::Impl
{
    Impl(const RefSocketor &client, std::string agentName);

    [[noreturn]]void MAIN();

    std::string agentName;

    Log logger;

    RefSocketor client;
};


Agent::Impl::Impl(const RefSocketor &client, std::string agentName) : logger("Agent_"s + Impl::agentName,
                                                                             LogFlag::CONSOLE_CLIENT,
                                                                             client),
                                                                    agentName(std::move(agentName)),
                                                                    client(client)
{
}

void Agent::MAIN()
{
    if (!implPtr) [[unlikely]]
        throw SDKException("Agent::MAIN() implPtr is nullptr");
    implPtr->MAIN();
}


void Agent::Impl::MAIN()
{
    logger.info("Agent::MAIN: {} start", agentName);
    logger.info("next operation no implement");
    exit(0);
}


Agent::Agent(const RefSocketor &client, const std::string &agentName) : implPtr(new Impl(client, agentName))
{
}

Agent::~Agent()
{
    delete implPtr;
}


