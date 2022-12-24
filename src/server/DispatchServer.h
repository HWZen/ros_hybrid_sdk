//
// Created by HWZ on 2022/11/2.
//

#ifndef ROS_HYBIRD_SDK_DISPATCHSERVER_H
#define ROS_HYBIRD_SDK_DISPATCHSERVER_H

class DispatchServer
{
public:
    DispatchServer() = default;

    void init(unsigned short int port = 5150);

    void run();

    ~DispatchServer();
private:
    struct Impl;
    Impl *implPtr{};
};

#endif // ROS_HYBIRD_SDK_DISPATCHSERVER_H
