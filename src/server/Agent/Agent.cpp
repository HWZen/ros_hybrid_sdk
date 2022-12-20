//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "../Log.h"
#include "../../SDKException.h"
#include "../CommandMsg/Command.pb.h"
#include "../MsgLoader.h"
#include "../asioHeader.h"
#include <sstl/thread.h>
#include <sys/prctl.h>
#include <ros/ros.h>

using namespace std::string_literals;
using namespace std::chrono_literals;

struct Agent::Impl
{
    void MAIN();

    std::string agentName{};

    std::shared_ptr<Log> logger{};

    int fd{};
    RefSocketor client{};

    int pipFd{};

    awaitable<void> parseCommand(const std::string &commandStr);

    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgPublisher>> pubMap{};
    std::unordered_map<std::string, std::shared_ptr<hybrid::MsgSubscriber>> subMap{};

    sstd::BaseThread *rosSpinThread{nullptr};

    ~Impl();


};

void Agent::MAIN()
{
    implPtr->MAIN();
}

extern int g_argc;
extern char **g_argv;

void Agent::Impl::MAIN()
{
start:
    logger->debug("preBootAgent start");
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
            return;
        }

        fd = *(int *)CMSG_DATA((cmsghdr *)&cmBuf);
        logger->debug("recv a socket, fd: {}", fd);
    }

    /*******************
     * recv agent name
     *******************/
    {
        std::array<char, 128> agentNameBuf{};
        auto len = read(pipFd, agentNameBuf.data(), agentNameBuf.size());
        if (len == -1) {
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            throw SDKException(fmt::format("recv agent name error, code: {}, reason: {}", errCode, reason));
        }

        agentName = std::string(agentNameBuf.data(), len);
        logger->debug("recv agent name: {}", agentName);
    }

    /*******************
     * fork, as another preBootAgent
     ******************/
    {
        auto pid = fork();
        if (pid == -1) {
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            throw SDKException(fmt::format("fork error, code: {}, reason: {}", errCode, reason));
        }
        if (pid == 0) {
            // child
            goto start;
        }
        else{
            // parent
            // close pip
            close(pipFd);
        }
    }


    /*******************
     * init
     *******************/

    asio::io_context ctx{};
    client = make_socket(ctx, asio::ip::tcp::v4(), fd);
    Impl::logger = std::make_shared<Log>("Agent_" + agentName, LogFlag::CONSOLE_CLIENT, client);

    auto &logger = *Impl::logger;
    // change process name
    logger.debug("change process name");
    g_argv[0] = agentName.data();
    prctl(PR_SET_NAME, agentName.c_str(), 0, 0, 0);



    logger.debug("init ros");
    ros::init(g_argc, g_argv, agentName);
    ros::start();

    logger.debug("start ros spin thread");
    rosSpinThread = new sstd::thread([&]() {
        ros::spin();
        logger.debug("ros spin thread exit");
    });

    logger.debug("spawn task: recv command");
    co_spawn(ctx, [&]() -> awaitable<void> {
        try {
            std::string read_buffer;
            for (;;) {
                auto delimiter = HYBRID_DELIMITER;
                auto [ec, len] = co_await asio::async_read_until(*client,
                                                                 asio::dynamic_buffer(read_buffer, 4096),
                                                                 delimiter,
                                                                 use_nothrow_awaitable);

                if (ec && ec != asio::error::eof) {
                    logger.error("read error: {}", ec.message());
                    ctx.stop();
                    exit(1);
                }
                if (ec == asio::error::eof) {
                    logger.info("disconnect");
                    ctx.stop();
                    co_return ;
                }

                co_await parseCommand(read_buffer.substr(0, len - HYBRID_DELIMITER_SIZE));
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

awaitable<void> Agent::Impl::parseCommand(const std::string &commandStr)
{
    auto & logger = *Impl::logger;
    hybrid::Command command;
    if (!command.ParseFromString(commandStr))
        logger.error("parse command error");
//    logger.debug("command: {}", command.DebugString());

    switch (command.type()) {
    case hybrid::Command_Type_UNKNOWN:break;
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
                                                                      advertise.has_queue_size() ? advertise.queue_size() : 100,
                                                                      advertise.has_latch() && advertise.latch()));
            logger.info("advertise topic: {}", advertise.topic());
        }
        catch (std::runtime_error &e) {
            logger.error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_PUBLISH:{
        if (!command.has_publish()) {
            logger.error("publish data not found");
            break;
        }
        const auto &publish = command.publish();
        if (pubMap.count(publish.topic()) == 0) {
            logger.error("topic {} not found, please advertise it first", publish.topic());
            break;
        }
        try{
            pubMap[publish.topic()]->publish(publish.data());
        }
        catch (std::runtime_error &e) {
            logger.error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNADVERTISE:{
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
    case hybrid::Command_Type_SUBSCRIBE:{
        if (!command.has_subscribe()) {
            logger.error("subscribe data not found");
            break;
        }
        const auto &subscribe = command.subscribe();
        if (subMap.count(subscribe.topic()) > 0) {
            logger.error("topic {} already exist", subscribe.topic());
            break;
        }
        try{
            logger.info("subscribe topic: {}", subscribe.topic());
            auto subscriber_maker = MsgLoader::getSubscriber(subscribe.type());
            subMap[subscribe.topic()] =
                std::shared_ptr<hybrid::MsgSubscriber>(subscriber_maker(subscribe.topic(),
                                                                        subscribe.has_queue_size() ? subscribe.queue_size() : 100,
                                                                        [&, subscribe](const std::string &msg)
                                                                        {
                                                                            hybrid::Command command;
                                                                            command.set_type(hybrid::Command_Type_SUBSCRIBE);
                                                                            auto &resSub = *command.mutable_subscribe();
                                                                            resSub.set_topic(subscribe.topic());
                                                                            resSub.set_type(subscribe.type());
                                                                            resSub.set_data(msg);
                                                                            client->async_write_some(buffer(resSub.SerializeAsString()), [&](const asio::error_code &ec, size_t) {
                                                                                if (ec)
                                                                                    logger.error("send msg error: {}", ec.message());
                                                                            });
                                                                        }
                                                                        ));
        }
        catch (std::runtime_error &e) {
            logger.error("subscribe exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNSUBSCRIBE:{
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
        logger.error("unknown command");
    }
    co_return ;

}
Agent::Impl::~Impl()
{
    logger->debug("agent exit");
    ros::shutdown();
    rosSpinThread->join();
}
