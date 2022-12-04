//
// Created by HWZ on 2022/11/3.
//

#include "Log.h"
#include "SDKException.h"
#include <ros/console.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <unistd.h>
#include <sstl/thread.h>


class ros_log_sink : public spdlog::sinks::base_sink<spdlog::details::null_mutex>
{
public:
    ~ros_log_sink() override = default;

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override;

    void flush_() override {}
};

class client_sink : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit client_sink(ref_client client) : client(std::move(client)) {
        // json pattern
        set_pattern(R"({"type":"log", "log": { "time": "%Y-%m-%d %H:%M:%S.%e" ,"level": "%^%l%$", "msg": "%v"}})");
    }

    ~client_sink() override = default;

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override;

    void flush_() override {}

    ref_client client;
};

void client_sink::sink_it_(const spdlog::details::log_msg &msg)
{
    if (!should_log(msg.level))
        return;
    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
    client->async_write_some(asio::buffer(formatted.data(), formatted.size()), [](const asio::error_code &ec, size_t) {
        if (ec)
            throw std::runtime_error(ec.message());
    });
}


struct Log::Impl : public sstd::ref_ptr<spdlog::logger>
{
    explicit Impl(sstd::ref_ptr<spdlog::logger> &&logger) : sstd::ref_ptr<spdlog::logger>(
            std::move(logger)) { (*this)->set_level(spdlog::level::trace); }
};


void ros_log_sink::sink_it_(const spdlog::details::log_msg &msg)
{
    if (!should_log(msg.level))
        return;
    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
    int level = msg.level == 0 ? 0 : msg.level - 1;
    ROS_LOG((ros::console::Level) level, ROSCONSOLE_DEFAULT_NAME, "%s", formatted.data());
}

spdlog::sink_ptr g_ros_sink = std::make_shared<ros_log_sink>();
spdlog::sink_ptr g_stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
spdlog::sink_ptr g_file_sink = nullptr;

void Log::log(int level, const std::string &msg)
{
    auto &impl = *implPtr;
    impl->log((spdlog::level::level_enum) level, msg);
}

void Log::init()
{
    // TODO: config
    g_file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("ros_hybird_sdk.log", 1024 * 1024 * 10, 3);
    g_file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e]  %8l:   %v");
    g_file_sink->set_level(spdlog::level::trace);
    g_stdout_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%L] P:%-6P t:%-6t %28n: %$  %v");
    g_stdout_sink->set_level(spdlog::level::debug);
    g_ros_sink->set_pattern("P:%-6P t:-6%t %28n: %v");
    g_ros_sink->set_level(spdlog::level::info);
    // TODO: client sink config
}

Log::Log(const std::string &name, LogFlag flag, const ref_client &client)
{
    std::vector<spdlog::sink_ptr> sinks;
    if (flag & LogFlag::CONSOLE_LOGGER)
        sinks.push_back(g_stdout_sink);
    if (flag & LogFlag::ROS_LOGGER)
        sinks.push_back(g_ros_sink);
    if (flag & LogFlag::FILE_LOGGER) {
        if (!g_file_sink)
            throw SDKException("File log is not initialized");
        sinks.push_back(g_file_sink);
    }
    if (flag & LogFlag::CLIENT_LOGGER) {
        if (!client)
            throw SDKException("Client is not initialized");
        sinks.push_back(std::make_shared<client_sink>(client));
    }

    implPtr = new Impl(sstd::ref_ptr{new spdlog::logger(name, sinks.begin(), sinks.end())});
}
