//
// Created by HWZ on 2022/11/3.
//

#ifndef ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_LOG_H
#define ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_LOG_H

#include <string>
#include "RefSocketor.h"
#include "spdlog/fmt/bundled/format.h"

using fmt::v8::format_string;

enum class LogFlag : int
{
    CONSOLE_LOGGER = 0X01,
    FILE_LOGGER = 0X02,
    ROS_LOGGER = 0X04,
    CLIENT_LOGGER = 0X08,
    CONSOLE_CLIENT = CONSOLE_LOGGER | CLIENT_LOGGER,

};

constexpr inline auto operator|(const LogFlag &a, const LogFlag &b)
{
    return static_cast<LogFlag>(((int)a | (int)b));
}

constexpr inline auto operator&(const LogFlag &a, const LogFlag &b)
{
    return (int)a & (int)b;
}

using ref_client = RefSocketor;

class Log
{
public:
    Log(std::string_view name, LogFlag flag = LogFlag::CONSOLE_LOGGER, const ref_client &client = nullptr);

    void log(int level, std::string_view msg);

    template<class ...Args>
    void log(int level, format_string<Args...> format, Args &&...args);

    template<class ...Args>
    void debug(format_string<Args...> format, Args &&...args);

    template<class ...Args>
    void info(format_string<Args...> format, Args &&...args);

    template<class ...Args>
    void warn(format_string<Args...> format, Args &&...args);

    template<class ...Args>
    constexpr void error(format_string<Args...> format, Args &&...args);

    template<class ...Args>
    void critical(format_string<Args...> format, Args &&...args);

    static void init();

    ~Log();

private:
    struct Impl;
    Impl *implPtr{};
};

/*****************************
 * Implementation
 ***************************/

template<class ...Args>
void Log::debug(format_string<Args...> format, Args &&...args)
{
    log(1, format, std::forward<decltype(args)>(args)...);
}

template<class ...Args>
void Log::info(format_string<Args...> format, Args &&...args)
{
    log(2, format, std::forward<decltype(args)>(args)...);
}

template<class ...Args>
void Log::warn(format_string<Args...> format, Args &&...args)
{
    log(3, format, std::forward<decltype(args)>(args)...);
}

template<class ...Args>
constexpr void Log::error(format_string<Args...> format, Args &&...args)
{
    log(4, format, std::forward<decltype(args)>(args)...);
}

template<class ...Args>
void Log::critical(format_string<Args...> format, Args &&...args)
{
    log(5, format, std::forward<decltype(args)>(args)...);
}

template<class ...Args>
void Log::log(int level, format_string<Args...> format, Args &&...args)
{
    log(level, fmt::format(format, std::forward<decltype(args)>(args)...));
}

#endif //ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_LOG_H
