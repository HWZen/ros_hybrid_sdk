//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "../Log.h"
#include "../../SDKException.h"
#include "../protoData/Command/Command.pb.h"
#include "../MsgLoader.h"
#include "../HybridSpinner.h"
#include <sstl/thread.h>
#include <sys/prctl.h>
#include <ros/ros.h>
#include <google/protobuf/util/json_util.h>

using namespace std::string_literals;
using namespace std::chrono_literals;

struct Agent::Impl
{
    void MAIN();

    std::shared_ptr<Log> logger{};

    int fd{};
    RefSocketor client{};

    int pipFd{};

    awaitable<void> parseCommand(std::string_view commandStr);

    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgPublisher>> pubMap{};
    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgSubscriber>> subMap{};

    ros::CallbackQueue topicQueue{};
    ros::CallbackQueue serviceQueue{};

    ~Impl();

};

void Agent::MAIN()
{
    if (!implPtr)
        throw SDKException("Agent::MAIN() implPtr is nullptr");
    implPtr->MAIN();
}

extern int g_argc;
extern char **g_argv;

void Agent::Impl::MAIN()
{
    logger->info("preBootAgent process start");
    sstd::thread checkParentThread([&]()
                                   {
                                       pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, nullptr);
                                       while (true) {
                                           if (getppid() == 1) {
                                               logger->info("parent process is dead, exit");
                                               exit(0);
                                           }
                                           sstd::ThisThread::sleep(1000);
                                           pthread_testcancel();
                                       }
                                   });
start:
    /*******************
     * wait a socket
     *******************/

    {
        iovec iov[1];
        msghdr msg{};
        char buff[0];

        iov[0].iov_base = buff;
        iov[0].iov_len = 1;

        msg.msg_name = nullptr;
        msg.msg_namelen = 0;

        msg.msg_iov = iov;
        msg.msg_iovlen = 1;

        char cmBuf[CMSG_LEN(sizeof(SOCKET))];

        msg.msg_control = cmBuf;
        msg.msg_controllen = CMSG_LEN(sizeof(SOCKET));

        if (recvmsg(pipFd, &msg, 0) == -1) {
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            logger->error("recv socket error, code: {}, reason: {}", errCode, reason);
            goto start;
        }

        fd = *(int *)CMSG_DATA((cmsghdr *)&cmBuf);
        logger->debug("recv a socket, fd: {}", fd);
    }

    /*******************
     * recv agent config
     *******************/
    hybrid::AgentConfig agentConfig{};
    {
        std::array<char, 4096> agentNameBuf{};
        auto len = read(pipFd, agentNameBuf.data(), agentNameBuf.size());
        if (len == -1) {
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            logger->error("recv agent name error, code: {}, reason: {}", errCode, reason);
            goto start;
        }

        if (!agentConfig.ParseFromArray(agentNameBuf.data(), static_cast<int>(len))) {
            logger->error("parse agent config error");
            goto start;
        }

        logger->debug("recv agent config: {}", agentConfig.DebugString());
    }

    /*******************
     * fork, as normal agent
     ******************/
    {
        auto pid = fork();
        if (pid == -1) {
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            throw SDKException(fmt::format("fork error, code: {}, reason: {}", errCode, reason));
        }
        if (pid != 0) {
            // parent
            // close fd
            close(fd);
            goto start;
        } else {
            // child
            // close pip
            close(pipFd);
            // stop check parent thread
            checkParentThread.terminate();
        }
    }


    /*******************
     * init
     *******************/

    const auto &agentName = agentConfig.node();
    asio::io_context ctx{};
    client = make_client(ctx, asio::ip::tcp::v4(), fd);
    client->agentConfig = std::move(agentConfig);
    Impl::logger = std::make_shared<Log>("Agent_" + agentName, LogFlag::CONSOLE_CLIENT, client);
    auto topicSpinner = HybridSpinner(agentName + "TopicSpinner", sstd::getCpuNums() / 4);
    auto srvSpinner = HybridSpinner(agentName + "SrvSpinner", sstd::getCpuNums() / 4);

    auto &logger = *Impl::logger;
    // change process name
    logger.debug("change process name");
    g_argv[0] = client->agentConfig.mutable_node()->data();
    prctl(PR_SET_NAME, agentName.c_str(), 0, 0, 0);

    logger.debug("init ros");
    // check agentName is already exist or not?
    if (ros::isInitialized()) [[unlikely]] {
        logger.error("ros is already initialized");
        throw SDKException("ros is already initialized");
    }
    ros::init(g_argc, g_argv, agentName);
    ros::start();

    topicSpinner.spin(&topicQueue);
    srvSpinner.spin(&serviceQueue);

    // check parent process if exit
    co_spawn(ctx, [&]() -> awaitable<void>
    {
        for (;;) {
            if (getppid() == 1) {
                this->logger->info("parent process is dead, exit");
                exit(0);
            }
            co_await timeout(1s);
        }
    }, asio::detached);

    /****************
     * main loop
     ***************/

    co_spawn(ctx, [&]() -> awaitable<void>
    {
        try {
            logger.info("login success");
            std::string read_buffer;
            for (;;) {
                auto [ec, len] = co_await asio::async_read_until(*client,
                                                                 asio::dynamic_buffer(read_buffer, 4096),
                                                                 client->agentConfig.delimiter(),
                                                                 use_nothrow_awaitable);

                if (ec && ec != asio::error::eof) {
                    logger.error("read error: {}", ec.message());
                    ctx.stop();
                    exit(1);
                }
                if (ec == asio::error::eof) {
                    logger.info("disconnect");
                    ctx.stop();
                    co_return;
                }

                co_await parseCommand(std::string_view(read_buffer.data(),
                                                       len - client->agentConfig.delimiter().size()));
                read_buffer.erase(0, len);
            }
        }
        catch (const std::exception &e) {
            logger.error("catch exception: ", e.what());
        }
    }, asio::detached);

    ctx.run();
    logger.debug("exit");

}

Agent::Agent(int pipFd) : implPtr(new Impl)
{
    implPtr->pipFd = pipFd;
    implPtr->logger = std::make_shared<Log>("PreBootAgent", LogFlag::CONSOLE_LOGGER);
}

Agent::~Agent()
{
    delete implPtr;
}

awaitable<void> Agent::Impl::parseCommand(std::string_view commandStr)
{
    auto &logger = *Impl::logger;
    hybrid::Command command;

    if (client->agentConfig.is_protobuf()) {
        if (!command.ParseFromArray(commandStr.data(), static_cast<int>(commandStr.size())))
            logger.error("parse command error");
    } else {
        if (auto state = google::protobuf::util::JsonStringToMessage(commandStr, &command); !state.ok())
            logger.error("parse command error: {}", state.ToString());
    }
//    logger.debug("command: {}", command.DebugString());

    switch (command.type()) {
    case hybrid::Command_Type_UNKNOWN:
        break;
    case hybrid::Command_Type_ADVERTISE: {
        if (!command.has_advertise()) {
            logger.error("advertise data not found");
            break;
        }
        const auto &advertise = command.advertise();
        if (pubMap.count(advertise.topic()) > 0) {
            logger.error("topic {} already exist", advertise.topic());
            break;
        }

        try {
            auto publisher_maker = MsgLoader::getPublisher(advertise.type());
            pubMap[advertise.topic()] =
                std::shared_ptr<hybrid::MsgPublisher>(publisher_maker(advertise.topic(),
                                                                      advertise.has_queue_size()
                                                                      ? advertise.queue_size() : 100,
                                                                      &topicQueue,
                                                                      client->agentConfig.is_protobuf(),
                                                                      advertise.has_latch() && advertise.latch()));
            logger.info("advertise topic: {}", advertise.topic());
        }
        catch (std::runtime_error &e) {
            logger.error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_PUBLISH: {
        if (!command.has_publish()) {
            logger.error("publish data not found");
            break;
        }
        if (command.mutable_publish()->has_string_data())
            command.mutable_publish()->set_data(command.mutable_publish()->string_data());
        const auto &publish = command.publish();
        if (pubMap.count(publish.topic()) == 0) {
            logger.error("topic {} not found, please advertise it first", publish.topic());
            break;
        }
        try {
            pubMap[publish.topic()]->publish(publish.data());
        }
        catch (std::runtime_error &e) {
            logger.error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNADVERTISE: {
        if (!command.has_unadvertise()) {
            logger.error("unadvertise data not found");
            break;
        }
        const auto &unadvertise = command.unadvertise();
        if (pubMap.count(unadvertise.topic()) == 0) {
            logger.error("topic {} not found", unadvertise.topic());
            break;
        }
        pubMap.erase(unadvertise.topic());
        logger.info("unadvertise topic: {}", unadvertise.topic());
        break;
    }
    case hybrid::Command_Type_SUBSCRIBE: {
        if (!command.has_subscribe()) {
            logger.error("subscribe data not found");
            break;
        }
        const auto &subscribe = command.subscribe();
        if (subMap.count(subscribe.topic()) > 0) {
            logger.error("topic {} already exist", subscribe.topic());
            break;
        }
        try {
            logger.info("subscribe topic: {}", subscribe.topic());
            auto subscriber_maker = MsgLoader::getSubscriber(subscribe.type());
            subMap[subscribe.topic()] =
                std::shared_ptr<hybrid::MsgSubscriber>(subscriber_maker(subscribe.topic(),
                                                                        subscribe.has_queue_size()
                                                                        ? subscribe.queue_size() : 100,
                                                                        &topicQueue,
                                                                        client->agentConfig.is_protobuf(),
                                                                        [&, subscribe](const std::string &msg)
                                                                        {
                                                                            hybrid::Command command;
                                                                            command.set_type(hybrid::Command_Type_PUBLISH);
                                                                            auto &resPub = *command.mutable_publish();
                                                                            resPub.set_topic(subscribe.topic());
                                                                            resPub.set_type(subscribe.type());
                                                                            resPub.set_data(msg);
                                                                            auto resString = command.SerializeAsString()
                                                                                + HYBRID_DELIMITER;
                                                                            client->async_write_some(buffer(resString),
                                                                                                     [&](const asio::error_code &ec,
                                                                                                         size_t)
                                                                                                     {
                                                                                                         if (ec)
                                                                                                             logger.error(
                                                                                                                 "send msg error: {}",
                                                                                                                 ec.message());
                                                                                                     });
                                                                        }
                ));
        }
        catch (std::runtime_error &e) {
            logger.error("subscribe exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNSUBSCRIBE: {
        if (!command.has_unsubscribe()) {
            logger.error("unsubscribe data not found");
            break;
        }
        const auto &unsubscribe = command.unsubscribe();
        if (subMap.count(unsubscribe.topic()) == 0) {
            logger.error("topic {} not found", unsubscribe.topic());
            break;
        }
        subMap.erase(unsubscribe.topic());
        logger.info("unsubscribe topic: {}", unsubscribe.topic());
        break;
    }
    case hybrid::Command_Type_ADVERTISE_SERVICE:
    case hybrid::Command_Type_CALL_SERVICE:
    case hybrid::Command_Type_RESPONSE_SERVICE:
    case hybrid::Command_Type_UNADVERTISE_SERVICE:
    case hybrid::Command_Type_LOG:
    case hybrid::Command_Type_PING:
    case hybrid::Command_Type_Command_Type_INT_MIN_SENTINEL_DO_NOT_USE_:
    case hybrid::Command_Type_Command_Type_INT_MAX_SENTINEL_DO_NOT_USE_:
        logger.info("command not implement");
        break;
    default:
        logger.error("unknown command: {}", command.type());
    }
    co_return;

}
Agent::Impl::~Impl()
{
    logger->debug("agent exit");
    ros::shutdown();
}
