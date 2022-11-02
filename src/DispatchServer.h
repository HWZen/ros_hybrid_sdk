//
// Created by HWZ on 2022/11/2.
//

#ifndef ROS_HYBIRD_SDK_DISPATCHSERVER_H
#define ROS_HYBIRD_SDK_DISPATCHSERVER_H

#include <cstdint>
class DispatchServer
{
public:
    DispatchServer() = default;

    void init(uint16_t port = 5150);

    void run();

    ~DispatchServer();
private:
    struct Impl;
    Impl* impl_{};
};

#endif // ROS_HYBIRD_SDK_DISPATCHSERVER_H
