//
// Created by HWZ on 2022/11/3.
//

#include "Agent.h"
#include "../Log.h"
#include "../../SDKException.h"
#include "../protoData/Command/Command.pb.h"
#include "../MsgLoader.h"
#include "../HybridSpinner.h"
#include <asio/write.hpp>
#include <sstl/thread.h>
#include <sys/prctl.h>
#include <ros/ros.h>
#include <google/protobuf/util/json_util.h>
#include <semaphore>

using namespace std::string_literals;
using namespace std::chrono_literals;

static constexpr auto g_serviceTimeout = 10s;


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

    struct ServiceData {
        std::shared_ptr<hybrid::SrvAdvertiser> advertiser;
        std::atomic_uint64_t seq{0};
        std::unordered_map<uint64_t, std::pair<std::shared_ptr<std::binary_semaphore>, std::string>> reqMap{};
    };

    std::unordered_map<std::string, ServiceData> srvServerMap{};
    std::unordered_map<std::string, std::shared_ptr<hybrid::SrvCaller>> srvClientMap{};


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
            logger.info("login success");
            std::string read_buffer;

            for (;;) {
                try {
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
                catch (std::exception &e) {
                    logger.error("catch exception: ", e.what());
                }
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
        // TODO: 不再使用std::map::count, 使用find，能够减少一次查找
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
                std::shared_ptr<hybrid::MsgSubscriber>(
                    subscriber_maker(subscribe.topic(),
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
                                         auto resString = command.SerializeAsString() + client->agentConfig.delimiter();
                                         client->async_write_some(buffer(resString),
                                                                  [&](const asio::error_code &ec, size_t)
                                                                  {
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
    case hybrid::Command_Type_ADVERTISE_SERVICE: {
        if (!command.has_advertise_service()) {
            logger.error("advertise service data not found");
            break;
        }
        const auto &advertiseService = command.advertise_service();
        if (srvServerMap.count(advertiseService.service()) > 0) {
            logger.error("service {} already exist", advertiseService.service());
            break;
        }
        try {
            auto server_maker = MsgLoader::getSeriviceServer(advertiseService.type());
            auto &serverData = srvServerMap[advertiseService.service()];
            serverData.advertiser =
                std::shared_ptr<hybrid::SrvAdvertiser>(
                    server_maker(advertiseService.service(),
                                 &serviceQueue,
                                 client->agentConfig.is_protobuf(),
                                 [serverName = advertiseService.service(),
                                     this,
                                     &serverData]
                                     (const std::string &req) -> std::string
                                 {
                                     uint64_t seq = serverData.seq++;
                                     asio::error_code ec;
                                     hybrid::Command command;
                                     command.set_type(hybrid::Command_Type_CALL_SERVICE);
                                     auto &callService = *command.mutable_call_service();
                                     callService.set_service(serverName);
                                     callService.set_seq(seq);
                                     if (client->agentConfig.is_protobuf()) {
                                         callService.set_data(req);
                                         asio::write(*client,
                                                     asio::buffer(command.SerializeAsString()),
                                                     asio::transfer_all(),
                                                     ec);
                                     }
                                     else {
                                         callService.set_string_data(req);
                                         std::string resBuf;
                                         auto res = google::protobuf::util::MessageToJsonString(command, &resBuf);
                                         if (!res.ok()) {
                                             this->logger->error("protobuf to json error, reason: {}", res.ToString());
                                             return {};
                                         }
                                         asio::write(*client,
                                                     asio::buffer(resBuf + client->agentConfig.delimiter()),
                                                     asio::transfer_all(),
                                                     ec);
                                     }
                                     if (ec) {
                                         this->logger->error("send msg error: {}", ec.message());
                                         return {};
                                     }
                                     auto sem = std::make_shared<std::binary_semaphore>(0);
                                     auto &reqPair = serverData.reqMap[seq];
                                     reqPair.first = sem;
                                     reqPair.second = {};
                                     if (!sem->try_acquire_for(g_serviceTimeout)) {
                                         this->logger->warn("wait response timeout, service: {}, seq: {}",
                                                             serverName,
                                                             seq);
                                         serverData.reqMap.erase(seq);
                                         return {};
                                     }
                                     auto res = std::move(serverData.reqMap[seq].second);
                                     serverData.reqMap.erase(seq);
                                     return res;
                                 }
                    ));
            logger.info("advertise service: {}", advertiseService.service());
        }
        catch (std::runtime_error &e) {
            logger.error("advertise service exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_RESPONSE_SERVICE:{
        if (!command.has_response_service()) {
            logger.error("response service data not found");
            break;
        }
        const auto &responseService = command.response_service();
        if (srvServerMap.count(responseService.service()) == 0) {
            logger.error("service {} not found", responseService.service());
            break;
        }
        auto &serverData = srvServerMap[responseService.service()];
        if (serverData.reqMap.count(responseService.seq()) == 0) {
            logger.error("service {} seq {} not found", responseService.service(), responseService.seq());
            break;
        }
        auto &reqPair = serverData.reqMap[responseService.seq()];
        if (client->agentConfig.is_protobuf())
            reqPair.second = responseService.data();
        else
            reqPair.second = responseService.string_data();
        reqPair.first->release();
        break;
    }
    case hybrid::Command_Type_CALL_SERVICE:{
        hybrid::Command responseCommand;
        responseCommand.set_type(hybrid::Command_Type_RESPONSE_SERVICE);
        auto &responseService = *responseCommand.mutable_response_service();
        responseService.set_success(false);
        std::string res;
        [&](){
            if (!command.has_call_service()) {
                logger.error("call service data not found");
                responseService.set_error_message("call service data not found");
                return;
            }
            const auto &callService = command.call_service();
            auto &serverCaller = srvClientMap[callService.service()];
            if (serverCaller == nullptr) {
                auto client_maker = MsgLoader::getServiceClient(callService.type());
                serverCaller = std::shared_ptr<hybrid::SrvCaller>(client_maker(
                    callService.service(),
                    &serviceQueue,
                    client->agentConfig.is_protobuf()));
            }
            // TODO: 加入处理队列
            try{
                if (client->agentConfig.is_protobuf())
                    res = serverCaller->call(callService.data());
                else
                    res = serverCaller->call(callService.string_data());
            }
            catch(std::runtime_error &e){
                logger.error("call service {} error: {}", callService.service(), e.what());
                responseService.set_error_message("call service error: "s + e.what());
                return;
            }
            if (res.empty()) {
                logger.error("call service {} failed", callService.service());
                responseService.set_error_message("call service failed");
                return;
            }
            responseService.set_success(true);
            responseService.set_service(callService.service());
            responseService.set_seq(callService.seq());
        }();

        if (client->agentConfig.is_protobuf()){
            responseService.set_data(res);
            co_await asio::async_write(*client,
                                       asio::buffer(responseCommand.SerializeAsString() + client->agentConfig.delimiter()),
                                       asio::use_awaitable);
        }
        else {
            responseService.set_string_data(res);
            std::string resBuf;
            auto status = google::protobuf::util::MessageToJsonString(responseCommand, &resBuf);
            if (!status.ok()) {
                this->logger->error("protobuf to json error, reason: {}", status.ToString());
                break;
            }
            co_await asio::async_write(*client,
                                       asio::buffer(resBuf + client->agentConfig.delimiter()),
                                       asio::use_awaitable);
        }
        break;
    }
    case hybrid::Command_Type_UNADVERTISE_SERVICE:{
        if (!command.has_unadvertise_service()) {
            logger.error("unadvertise service data not found");
            break;
        }
        const auto &unadvertiseService = command.unadvertise_service();
        if (auto it = srvServerMap.find(unadvertiseService.service()); it == srvServerMap.end()) {
            logger.error("service {} not found", unadvertiseService.service());
        }
        else {
            srvServerMap.erase(it);
            logger.info("unadvertise service: {}", unadvertiseService.service());
        }
        break;
    }
    case hybrid::Command_Type_LOG:{
        if (!command.has_log()) {
            logger.error("log data not found");
            break;
        }
        const auto &log = command.log();
        this->logger->log(log.level(), log.message());
        break;
    }
    case hybrid::Command_Type_PING:
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
