//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "Log.h"

struct Agent::Impl
{
    Impl();
    Impl(const RefSocketor& client);

    [[noreturn]]void MAIN();

    Log logger;
};


Agent::Impl::Impl(const RefSocketor& client) : logger("Agent",
                                               static_cast<LogFlag>((LogFlag::CONSOLE_LOGGER | LogFlag::CLIENT_LOGGER)),
                                               client)
{
}

void Agent::MAIN()
{
    implPtr->MAIN();
}

Agent::Impl::Impl() : logger("Agent")
{
}

void Agent::Impl::MAIN()
{
    logger.info("Agent::MAIN");
    exit(0);
}


Agent::Agent(const RefSocketor& client) : implPtr(new Impl(client))
{
}


