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
#include <asio/thread_pool.hpp>
#include <sstl/thread.h>
#include <sys/prctl.h>
#include <ros/ros.h>
#include <google/protobuf/util/json_util.h>
#include <semaphore>

using namespace std::string_literals;
using namespace std::chrono_literals;

static auto topicThreadNums = [](){auto res = sstd::getCpuNums() / 6; return res == 0 ? 1 : res; }();
static auto srvThreadNums = [](){auto res = sstd::getCpuNums() / 4; return res == 0 ? 1 : res; }();
static auto coroThreadNums = [](){auto res = sstd::getCpuNums() / 8; return res == 0 ? 1 : res; }();

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


    std::unique_ptr<asio::thread_pool> serviceCallPool{nullptr};
    template<asio::completion_token_for<void(bool, std::string)> CompletionToken>
    auto async_call_server(std::shared_ptr<hybrid::SrvCaller> serviceCaller,
                           const std::string &callData,
                           CompletionToken &&token);

    awaitable<void> call_server(hybrid::Command command);


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
        std::array<char, 4096> agentConfigBuf{};
        auto len = read(pipFd, agentConfigBuf.data(), agentConfigBuf.size());
        if (len == -1) {
            auto errCode = errno;
            auto reason = hstrerror(errCode);
            logger->error("recv agent name error, code: {}, reason: {}", errCode, reason);
            goto start;
        }

        if (!agentConfig.ParseFromArray(agentConfigBuf.data(), static_cast<int>(len))) {
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
    auto topicSpinner = HybridSpinner(agentName + "TopicSpinner", topicThreadNums);
    auto srvSpinner = HybridSpinner(agentName + "SrvSpinner", srvThreadNums);

    
    // change process name
    logger->debug("change process name");
    g_argv[0] = client->agentConfig.mutable_node()->data();
    prctl(PR_SET_NAME, agentName.c_str(), 0, 0, 0);

    logger->debug("init ros");
    // check agentName is already exist or not?
    if (ros::isInitialized()) [[unlikely]] {
        logger->error("ros is already initialized");
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

    serviceCallPool = std::make_unique<asio::thread_pool>(srvThreadNums);

    asio::post(*serviceCallPool, [&]()
               {
                   logger->debug("service call pool ready");
               });

    /****************
     * main loop
     ***************/

    co_spawn(ctx, [&]() -> awaitable<void>
    {
            logger->info("login success");
            std::string read_buffer;

            for (;;) {
                try {
                    auto [ec, len] = co_await asio::async_read_until(*client,
                                                                     asio::dynamic_buffer(read_buffer, 4096),
                                                                     client->agentConfig.delimiter(),
                                                                     use_nothrow_awaitable);
                    if (ec && ec != asio::error::eof) {
                        logger->error("read error: {}", ec.message());
                        ctx.stop();
                        exit(1);
                    }
                    if (ec == asio::error::eof) {
                        logger->info("disconnect");
                        ctx.stop();
                        co_return;
                    }

                    co_await parseCommand(std::string_view(read_buffer.data(),
                                                           len - client->agentConfig.delimiter().size()));
                    read_buffer.erase(0, len);
                }
                catch (std::exception &e) {
                    logger->error("catch exception: ", e.what());
                }
            }
    }, asio::detached);




    asio::thread_pool pool{coroThreadNums};
    auto threadFunc = [&]() {
        ctx.run();
    };
    for (size_t i = 0; i < coroThreadNums; ++i)
        asio::post(pool, threadFunc);
    pool.join();
    logger->debug("exit");

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

template<asio::completion_token_for<void(bool, std::string)> CompletionToken>
auto Agent::Impl::async_call_server(std::shared_ptr<hybrid::SrvCaller> serviceCaller,
                                    const std::string &callData,
                                    CompletionToken &&token)
{
    auto init = [this](asio::completion_handler_for<void(bool, std::string)> auto handler,
                       std::shared_ptr<hybrid::SrvCaller> serviceCaller,
                       const std::string &callData)
    {
        auto work = asio::make_work_guard(handler);

        asio::post(*serviceCallPool, [
                       work = std::move(work),
                       handler = std::move(handler),
                       callData,
                       serviceCaller = std::move(serviceCaller)
                   ]() mutable
                   {
                       auto alloc = asio::get_associated_allocator(
                           handler, asio::recycling_allocator<void>());
                       auto [res, data] = [&serviceCaller, &callData]()
                       {
                           try {
                               auto resultMsg = serviceCaller->call(callData);
                               return std::make_pair(true, resultMsg);
                           }
                           catch (std::runtime_error &e) {
                               return std::make_pair(false, std::string(e.what()));
                           }
                       }();

                       asio::dispatch(work.get_executor(), asio::bind_allocator(alloc,
                                                                                [
                                                                                    handler = std::move(handler),
                                                                                    res,
                                                                                    data = std::move(data)
                                                                                ]() mutable
                                                                                {
                                                                                    std::move(handler)(res, std::move(data));
                                                                                }
                       ));
                   }
        );
    };

    return asio::async_initiate<CompletionToken, void(bool, std::string)>(init, token, serviceCaller, callData);
}


awaitable<void> Agent::Impl::parseCommand(std::string_view commandStr)
{
    
    hybrid::Command command;

    if (client->agentConfig.is_protobuf()) {
        if (!command.ParseFromArray(commandStr.data(), static_cast<int>(commandStr.size())))
            logger->error("parse command error");
    } else {
        if (auto state = google::protobuf::util::JsonStringToMessage(commandStr, &command); !state.ok())
            logger->error("parse command error: {}", state.ToString());
    }

    switch (command.type()) {
    case hybrid::Command_Type_UNKNOWN:
        break;
    case hybrid::Command_Type_ADVERTISE: {
        if (!command.has_advertise()) {
            logger->error("advertise data not found");
            break;
        }
        const auto &advertise = command.advertise();
        if (pubMap.count(advertise.topic()) > 0) {
            logger->error("topic {} already exist", advertise.topic());
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
            logger->info("advertise topic: {}", advertise.topic());
        }
        catch (std::runtime_error &e) {
            logger->error("publish exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_PUBLISH: {
        if (!command.has_publish()) {
            logger->error("publish data not found");
            break;
        }
        if (command.mutable_publish()->has_string_data())
            command.mutable_publish()->set_data(command.mutable_publish()->string_data());
        const auto &publish = command.publish();
        if (auto it = pubMap.find(publish.topic()); it == pubMap.end()) {
            logger->error("topic {} not found, please advertise it first", publish.topic());
        }
        else{
            try {
                it->second->publish(publish.data());
            }
            catch (std::runtime_error &e) {
                logger->error("publish exception: {}", e.what());
            }

        }
        break;
    }
    case hybrid::Command_Type_UNADVERTISE: {
        if (!command.has_unadvertise()) {
            logger->error("unadvertise data not found");
            break;
        }
        const auto &unadvertise = command.unadvertise();
        if (auto it = pubMap.find(unadvertise.topic()); it == pubMap.end()) {
            logger->error("topic {} not found", unadvertise.topic());
            break;
        }
        else
            pubMap.erase(it);
        logger->info("unadvertise topic: {}", unadvertise.topic());
        break;
    }
    case hybrid::Command_Type_SUBSCRIBE: {
        if (!command.has_subscribe()) {
            logger->error("subscribe data not found");
            break;
        }
        const auto &subscribe = command.subscribe();
        if (subMap.count(subscribe.topic()) > 0) {
            logger->error("topic {} already exist", subscribe.topic());
            break;
        }
        try {
            logger->info("subscribe topic: {}", subscribe.topic());
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
                                                                          logger->error("send msg error: {}", ec.message());
                                                                  });
                                     }
                    ));
        }
        catch (std::runtime_error &e) {
            logger->error("subscribe exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_UNSUBSCRIBE: {
        if (!command.has_unsubscribe()) {
            logger->error("unsubscribe data not found");
            break;
        }
        const auto &unsubscribe = command.unsubscribe();
        if (auto it = subMap.find(unsubscribe.topic()); it == subMap.end()) {
            logger->error("topic {} not found", unsubscribe.topic());
            break;
        }
        else
            subMap.erase(it);
        logger->info("unsubscribe topic: {}", unsubscribe.topic());
        break;
    }
    case hybrid::Command_Type_ADVERTISE_SERVICE: {
        if (!command.has_advertise_service()) {
            logger->error("advertise service data not found");
            break;
        }
        const auto &advertiseService = command.advertise_service();
        if (srvServerMap.count(advertiseService.service()) > 0) {
            logger->error("service {} already exist", advertiseService.service());
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
                                     auto res = std::move(reqPair.second);
                                     serverData.reqMap.erase(seq);
                                     return res;
                                 }
                    ));
            logger->info("advertise service: {}", advertiseService.service());
        }
        catch (std::runtime_error &e) {
            logger->error("advertise service exception: {}", e.what());
        }
        break;
    }
    case hybrid::Command_Type_RESPONSE_SERVICE:{
        if (!command.has_response_service()) {
            logger->error("response service data not found");
            break;
        }
        const auto &responseService = command.response_service();
        auto srvServerIt = srvServerMap.find(responseService.service());
        if (srvServerIt == srvServerMap.end()) {
            logger->error("service {} not found", responseService.service());
            break;
        }
        auto &serverData = srvServerIt->second;
        auto serverDataIt = serverData.reqMap.find(responseService.seq());
        if (serverDataIt == serverData.reqMap.end()) {
            logger->error("service {} seq {} not found", responseService.service(), responseService.seq());
            break;
        }
        auto &reqPair = serverDataIt->second;
        if (client->agentConfig.is_protobuf())
            reqPair.second = responseService.data();
        else
            reqPair.second = responseService.string_data();
        reqPair.first->release();
        break;
    }
    case hybrid::Command_Type_CALL_SERVICE:{
        co_spawn(co_await asio::this_coro::executor, call_server(std::move(command)), asio::detached);
        break;
    }
    case hybrid::Command_Type_UNADVERTISE_SERVICE:{
        if (!command.has_unadvertise_service()) {
            logger->error("unadvertise service data not found");
            break;
        }
        const auto &unadvertiseService = command.unadvertise_service();
        if (auto it = srvServerMap.find(unadvertiseService.service()); it == srvServerMap.end()) {
            logger->error("service {} not found", unadvertiseService.service());
        }
        else {
            srvServerMap.erase(it);
            logger->info("unadvertise service: {}", unadvertiseService.service());
        }
        break;
    }
    case hybrid::Command_Type_LOG:{
        if (!command.has_log()) {
            logger->error("log data not found");
            break;
        }
        const auto &log = command.log();
        this->logger->log(log.level(), log.message());
        break;
    }
    case hybrid::Command_Type_PING:
    default:
        logger->error("unknown command: {}", command.type());
    }
    co_return;

}
Agent::Impl::~Impl()
{
    logger->debug("agent exit");
    ros::shutdown();
}
awaitable<void> Agent::Impl::call_server(hybrid::Command command)
{
    hybrid::Command responseCommand;
    responseCommand.set_type(hybrid::Command_Type_RESPONSE_SERVICE);
    auto &responseService = *responseCommand.mutable_response_service();
    responseService.set_success(false);
    std::string res;
    co_await [&]() -> awaitable<void>{
        if (!command.has_call_service()) {
            this->logger->error("call service data not found");
            responseService.set_error_message("call service data not found");
            co_return;
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
        auto &callData = client->agentConfig.is_protobuf() ? callService.data() : callService.string_data();
        auto [success, data] = co_await async_call_server(serverCaller, callData, use_nothrow_awaitable);
        if (!success) {
            this->logger->error("call service {} error: {}", callService.service(), data);
            responseService.set_error_message("call service error: "s + data);
        }
        else if (data.empty()) {
            this->logger->error("call service {} failed", callService.service());
            responseService.set_error_message("call service failed");
        }
        else{
            res = data;
            responseService.set_success(true);
        }
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
            this->logger->error("{}@{}: protobuf to json error, reason: {}", __FILE__, __LINE__, status.ToString());
            co_return ;
        }
        co_await asio::async_write(*client,
                                   asio::buffer(resBuf + client->agentConfig.delimiter()),
                                   asio::use_awaitable);
    }
}
