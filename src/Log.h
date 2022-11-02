//
// Created by HWZ on 2022/11/3.
//

#ifndef ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_LOG_H
#define ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_LOG_H

#include <string>
#include "spdlog/fmt/bundled/format.h"

class Log
{
public:
    Log(const std::string &name);

    void log(int level, const std::string &msg);

    void log(int level, auto &&format, auto ...args);

    void debug(auto &&format, auto ...args);

    void info(auto &&format, auto ...args);

    void warn(auto &&format, auto ...args);

    void error(auto &&format, auto ...args);

    void critical(auto &&format, auto ...args);

private:
    struct Impl;
    Impl *impl;
};




/*****************************
 * Implementation
 ***************************/

void Log::debug(auto &&format, auto... args)
{
    log(1, std::forward<decltype(format)>(format), std::forward<decltype(args)>(args)...);
}

void Log::info(auto &&format, auto... args)
{
    log(2, std::forward<decltype(format)>(format), std::forward<decltype(args)>(args)...);
}

void Log::warn(auto &&format, auto... args)
{
    log(3, std::forward<decltype(format)>(format), std::forward<decltype(args)>(args)...);
}

void Log::error(auto &&format, auto... args)
{
    log(4, std::forward<decltype(format)>(format), std::forward<decltype(args)>(args)...);
}

void Log::critical(auto &&format, auto... args)
{
    log(5, std::forward<decltype(format)>(format), std::forward<decltype(args)>(args)...);
}

void Log::log(int level, auto &&format, auto ...args)
{
    log(level, fmt::format(std::forward<decltype(format)>(format), std::forward<decltype(args)>(args)...));
}


#endif //ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_LOG_H
