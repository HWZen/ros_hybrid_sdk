//
// Created by HWZ on 2022/11/3.
//

#include "Log.h"
#include "MsgWrapper/rosgraph_msgs/Log.h"
#include "../SDKException.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/base_sink.h"
#include "spdlog/sinks/rotating_file_sink.h"
#include "protoData/Command/Command.pb.h"
#include "asioHeader.h"
#include <unistd.h>
#include <google/protobuf/util/time_util.h>
#include <google/protobuf/util/json_util.h>


class ros_log_sink : public spdlog::sinks::base_sink<spdlog::details::null_mutex>
{
public:
    ~ros_log_sink() override = default;

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override;

    void flush_() override {}
};

class client_sink : public spdlog::sinks::base_sink<spdlog::details::null_mutex>
{
public:
    explicit client_sink(ref_client client);

    ~client_sink() override = default;

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override;

    void flush_() override {}

    ref_client client;

};

struct Log::Impl : public std::shared_ptr<spdlog::logger>
{
    explicit Impl(std::shared_ptr<spdlog::logger> &&logger) : std::shared_ptr<spdlog::logger>(std::move(logger))
    {
        (*this)->set_level(spdlog::level::trace);
    }
};


void ros_log_sink::sink_it_(const spdlog::details::log_msg &msg)
{
    if (!should_log(msg.level))
        return;
    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
//    ROS_LOG((ros::console::Level) level, ROSCONSOLE_DEFAULT_NAME, "%s", formatted.data());
    hybrid::Log log;

    auto getHeader = [](const std::string& frame_id = ""){
        static std::atomic_uint32_t seq{0};
        hybrid::Header header;
        header.seq = ++seq;
        header.stamp = ros::Time::now();
        header.frame_id = frame_id;
        return header;
    };
    log.header = getHeader();
    log.name = msg.logger_name.data();
    log.msg = {formatted.data(), formatted.size()};
    int level = msg.level == 0 ? 0 : msg.level - 1;
    log.level = level;
    static hybrid::LogPublisher pub("/rosout",1000);
    pub.publish(log);

}

void client_sink::sink_it_(const spdlog::details::log_msg &msg)
{
    if (!should_log(msg.level))
        return;
    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
    auto sinkCallback = [](const asio::error_code &ec, size_t) {
        if (ec)
            spdlog::error("client sink error: {}", ec.message());
    };
    hybrid::Command command;
    command.set_type(hybrid::Command::LOG);
    auto *log = command.mutable_log();
    log->set_level(static_cast<hybrid::Command_Log_Level>(msg.level == 0 ? 0 : msg.level - 1));
    *log->mutable_time() = google::protobuf::util::TimeUtil::GetCurrentTime();
    log->set_message(formatted.data(), formatted.size());
    if (client->agentConfig.is_protobuf()){
        auto buf = command.SerializeAsString() + HYBRID_DELIMITER;
        client->async_write_some(asio::buffer(buf), sinkCallback);
    }else{
        // json timestamp is UTC time. convert it to local time?
        std::string buf{};
        if (auto state = google::protobuf::util::MessageToJsonString(command, &buf); !state.ok())[[unlikely]]{
            spdlog::error("client sink error: {}", state.ToString());
            return;
        }
        buf += HYBRID_DELIMITER;
        client->async_write_some(asio::buffer(buf), sinkCallback);
    }
}

client_sink::client_sink(ref_client client) : client(std::move(client)){
    set_pattern("%v");
    set_level(static_cast<spdlog::level::level_enum>(client_sink::client->agentConfig.log_level()));
}


spdlog::sink_ptr g_ros_sink = std::make_shared<ros_log_sink>();
spdlog::sink_ptr g_stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
spdlog::sink_ptr g_file_sink = nullptr;

void Log::log(int level, const std::string &msg)
{
    assert(implPtr);
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
    g_ros_sink->set_pattern("P:%-6P t:%-6t %28n: %v");
    g_ros_sink->set_level(spdlog::level::info);
    spdlog::set_default_logger(spdlog::stdout_color_mt("Default"));
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%L] P:%-6P t:%-6t %28n: %$  %v");
    spdlog::set_level(spdlog::level::trace);
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

    implPtr = new Impl(std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end()));
}

Log::~Log()
{
    delete implPtr;
}
