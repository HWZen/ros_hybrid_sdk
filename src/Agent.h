//
// Created by HWZ on 2022/11/3.
//

#ifndef ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_AGENT_H
#define ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_AGENT_H

#include <cstdlib>
#include "RefSocketor.h"



class Agent
{
public:
    Agent() = default;
    Agent(const RefSocketor& client);

    [[noreturn]]void MAIN();

private:
    struct Impl;
    Impl *implPtr{};
};

#endif //ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_AGENT_H
