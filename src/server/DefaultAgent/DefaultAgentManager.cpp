//
// Created by HWZ on 2022/11/3.
//
#include "DefaultAgentManager.h"
#include "ConnectInstance.h"
#include "../Log.h"
#include "../HybridSpinner.h"
#include "GlobalVars.h"
#include <sstl/thread.h>
#include <ros/ros.h>
#include <unordered_set>
#include <semaphore>
#include <sys/prctl.h>
using namespace std::chrono_literals;

extern int g_argc;
extern char **g_argv;

struct DefaultAgentManager::Impl
{

    int recv_fd();

    Log logger{"DefaultAgentManager", LogFlag::CONSOLE_LOGGER | LogFlag::ROS_LOGGER};

    void MAIN();

    void setSocketPipe(int pipe);

    int pipeFd{};

    awaitable<void> listenFd();

    std::shared_ptr<sstd::any_thread> recvFdThread{};
    std::shared_ptr<std::function<void(SOCKET, hybrid::AgentConfig)>> task_ref;
    std::binary_semaphore task_sem{0};

    template<asio::completion_token_for<void(SOCKET, hybrid::AgentConfig)> CompletionToken>
    auto async_get_socket(CompletionToken &&token);

    asio::io_context ctx{};

    HybridSpinner topicSpinner{"DefaultTopicSpinner", g_topicThreadNums};
    HybridSpinner srvSpinner{"DefaultSrvSpinner", g_srvThreadNums};

    virtual ~Impl();

};

void DefaultAgentManager::MAIN()
{
    implPtr->MAIN();
}

DefaultAgentManager::DefaultAgentManager() : implPtr(new DefaultAgentManager::Impl)
{
}

void DefaultAgentManager::setSocketPipe(int pipe)
{
    implPtr->setSocketPipe(pipe);
}

DefaultAgentManager::~DefaultAgentManager()
{
    delete implPtr;
}

void DefaultAgentManager::Impl::MAIN()
{
    // change process name
    g_argv[0] = new char[]{"DefaultAgent"};
    prctl(PR_SET_NAME, "DefaultAgent", 0, 0, 0);

    ros::init(g_argc, g_argv, "DefaultAgent");
    ros::start();


    topicSpinner.spin(&g_topicQueue);
    srvSpinner.spin(&g_serviceQueue);

    g_serviceCallPool = std::make_unique<asio::thread_pool>(g_srvThreadNums);
    asio::post(*g_serviceCallPool, [&](){
        logger.debug("service call pool ready");
    });

    this->logger.info("DefaultAgent::MAIN");

    // check parent process if exit
    co_spawn(ctx, [&]() -> awaitable<void>
    {
        for (;;) {
            if (getppid() == 1) {
                this->logger.info("parent process is dead, exit");
                topicSpinner.stop();
                srvSpinner.stop();
                ctx.stop();
            }
            co_await timeout(100ms);
        }
    }, asio::detached);

    co_spawn(ctx, listenFd(), asio::detached);

    asio::thread_pool coro_pool{g_coroThreadNums};
    auto threadFunc = [&](){
        ctx.run();
    };
    for (auto i = 0; i < g_coroThreadNums; ++i)
        asio::post(coro_pool, threadFunc);
    coro_pool.join();
}

void DefaultAgentManager::Impl::setSocketPipe(int pipe)
{
    pipeFd = pipe;
    recvFdThread =
        std::shared_ptr<sstd::any_thread>(new sstd::any_thread(sstd::thread([this]
                                                           {
                                                               for (;;) {
                                                                   auto fd = recv_fd();
                                                                   std::array<char, 4096> agentConfigBuf{};
                                                                   auto len = read(pipeFd,
                                                                                   agentConfigBuf.data(),
                                                                                   agentConfigBuf.size());
                                                                   hybrid::AgentConfig agentConfig{};
                                                                   if (!agentConfig.ParseFromArray(agentConfigBuf.data(),
                                                                                                   static_cast<int>(len))) {
                                                                       this->logger.error("parse agent config fail");
                                                                       continue;
                                                                   }
                                                                   task_sem.acquire();
                                                                   auto task = task_ref;
                                                                   task_ref = nullptr;
                                                                   (*task)(fd, std::move(agentConfig));
                                                               }
                                                           })),
            // destroy thread
                                          [](sstd::any_thread *th)
                                          {
                                              if (th)
                                                  th->terminate();
                                              delete th;
                                          });

}

template<asio::completion_token_for<void(SOCKET, hybrid::AgentConfig)> CompletionToken>
auto DefaultAgentManager::Impl::async_get_socket(CompletionToken &&token)
{

    // Define a function object that contains the code to launch the asynchronous
    // operation. This is passed the concrete completion handler, followed by any
    // additional arguments that were passed through the call to async_initiate.
    auto init = [this](asio::completion_handler_for<void(SOCKET, hybrid::AgentConfig)> auto handler)
    {
        // According to the rules for asynchronous operations, we need to track
        // outstanding work against the handler's associated executor until the
        // asynchronous operation is complete.

        auto ref_handler = std::make_shared<decltype(handler)>(std::move(handler));

        auto work = asio::make_work_guard(*ref_handler);

        std::function<void(SOCKET, hybrid::AgentConfig)> callback = [
            work = std::move(work),
            ref_handler = std::move(ref_handler)
        ](SOCKET socketFd, hybrid::AgentConfig agentConfig) mutable
        {
            // Get the handler's associated allocator. If the handler does not
            // specify an allocator, use the recycling allocator as the default.
            auto alloc = asio::get_associated_allocator(
                *ref_handler, asio::recycling_allocator<void>());

            // Dispatch the completion handler through the handler's associated
            // executor, using the handler's associated allocator.
            asio::dispatch(work.get_executor(),
                           asio::bind_allocator(alloc,
                                                [
                                                    ref_handle = std::move(ref_handler),
                                                    socketFd,
                                                    agentConfig = std::move(agentConfig)
                                                ]() mutable
                                                {
                                                    std::move(*ref_handle)(socketFd, std::move(agentConfig));
                                                }));
        };
        task_ref = std::make_shared<std::function<void(SOCKET, hybrid::AgentConfig)>>(std::move(callback));
        task_sem.release();
    };

    // The async_initiate function is used to transform the supplied completion
    // token to the completion handler. When calling this function we explicitly
    // specify the completion signature of the operation. We must also return the
    // result of the call since the completion token may produce a return value,
    // such as a future.
    return asio::async_initiate<CompletionToken, void(SOCKET, hybrid::AgentConfig)>(
        init, // First, pass the function object that launches the operation,
        token);// then the completion token that will be transformed to a handler

}

int DefaultAgentManager::Impl::recv_fd()
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

    if (recvmsg(pipeFd, &msg, 0) == -1) {
        auto errCode = errno;
        auto reason = hstrerror(errCode);
        logger.error("recv error, code: {}, reason: {}", errCode, reason);
        return 0;
    }

    int fd = *(int *)CMSG_DATA((cmsghdr *)&cmBuf);
    return fd;
}

awaitable<void> DefaultAgentManager::Impl::listenFd()
{
    for (;;) {
        try {

            auto [fd, agentConfig] = co_await async_get_socket(asio::use_awaitable);

            RefSocketor client = make_client(co_await this_coro::executor, asio::ip::tcp::v4(), fd);
            client->agentConfig = std::move(agentConfig);
            logger.info("recv a socketor, address: {}, port: {}", client->remote_endpoint().address().to_string(),
                         client->remote_endpoint().port());
            logger.debug("agent config: {}", client->agentConfig.DebugString());

            auto executor = client->get_executor();
            co_spawn(executor, [this] (auto client) mutable -> awaitable<void>
            {
                try{
                    ConnectInstance instance{std::move(client)};
                    if(co_await instance.MAIN() != 0){
                        logger.warn("connect instance main not return 0");
                    }
                }
                catch(std::exception &e){
                    logger.error("catch exception: {}", e.what());
                }
                co_return ;
            }(std::move(client)), asio::detached);
        }
        catch (std::exception &e) {
            logger.error("catch exception in {}@{} : {}", __FILE__, __LINE__, e.what());
        }

    }

}
DefaultAgentManager::Impl::~Impl()
{
    logger.debug("DefaultAgentManager::Impl::~Impl()");
    topicSpinner.stop();
    srvSpinner.stop();
    ros::shutdown();
}


