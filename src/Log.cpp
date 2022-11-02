//
// Created by HWZ on 2022/11/3.
//

#include "Log.h"
#include <ros/console.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/sink.h>
#include <unistd.h>
#include <sstl/thread.h>
#include <sstl/ref_ptr.h>
#include <socketor.h>

using ref_client = sstd::ref_ptr<mysock::socketor>;

class ros_log_sink : public spdlog::sinks::sink
{
public:
    ~ros_log_sink() override = default;
    void log(const spdlog::details::log_msg &msg) override;
    void flush() override {}
    void set_pattern(const std::string &pattern) override {}
    void set_formatter(std::unique_ptr<spdlog::formatter> sink_formatter) override{}
};

class client_sink : public spdlog::sinks::

struct Log::Impl : public spdlog::logger
{
    Impl(const std::string &name, ref_client = nullptr);
};


void ros_log_sink::log(const spdlog::details::log_msg &msg)
{
    if (!should_log(msg.level))
        return;
    int level = msg.level == 0 ? 0 : msg.level - 1;
    ROS_LOG((ros::console::Level)level, msg.logger_name.data(), "[P: %d][T: %lu] %s",
            getpid(),
            sstd::ThisThread::getThreadFd(),
            msg.payload.data());
}

void Log::log(int level, const std::string &msg)
{
    impl->log((spdlog::level::level_enum)level, msg);
}
