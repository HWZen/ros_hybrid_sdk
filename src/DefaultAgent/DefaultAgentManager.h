//
// Created by HWZ on 2022/11/3.
//

#ifndef ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_DEFAULTAGENT_H
#define ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_DEFAULTAGENT_H

class DefaultAgentManager
{
public:
    DefaultAgentManager();

    [[noreturn]] void MAIN();

    void setSocketPipe(int pipe);

    virtual ~DefaultAgentManager();

private:
    struct Impl;
    Impl *implPtr;
};

#endif //ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_DEFAULTAGENT_H
