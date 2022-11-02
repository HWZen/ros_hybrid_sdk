//
// Created by HWZ on 2022/11/3.
//

#ifndef ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_DEFAULTAGENT_H
#define ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_DEFAULTAGENT_H

#include "Agent.h"

class DefaultAgent : public Agent
{
public:
    [[noreturn]] void MAIN();

    void setSocketPipe(int pipe);

private:
    struct Impl;
    Impl *impl;
};

#endif //ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_DEFAULTAGENT_H
