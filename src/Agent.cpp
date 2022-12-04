//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "Log.h"
#include "SDKException.h"

struct Agent::Impl
{
    Impl();
    Impl(const RefSocketor &client, std::string agentName);

    [[noreturn]]void MAIN();

    std::string agentName;

    Log logger;
};


Agent::Impl::Impl(const RefSocketor &client, std::string agentName) : logger("Agent",
                                                                             LogFlag::CONSOLE_CLIENT,
                                                                             client),
                                                                    agentName(std::move(agentName))
{
}

void Agent::MAIN()
{
    if (!implPtr) [[unlikely]]
        throw SDKException("Agent::MAIN() implPtr is nullptr");
    implPtr->MAIN();
}

Agent::Impl::Impl() : logger("Agent")
{
}

void Agent::Impl::MAIN()
{
    logger.info("Agent::MAIN: {} start", agentName);
    exit(0);
}


Agent::Agent(const RefSocketor &client, const std::string &agentName) : implPtr(new Impl(client, agentName))
{
}


