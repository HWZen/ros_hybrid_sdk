//
// Created by HWZ on 2022/11/3.
//
#include "DefaultAgentManager.h"
#include "ConnectInstance.h"
#include "../Log.h"
#include "../asioHeader.h"
#include "sstl/thread.h"
#include "sstl/atomic_queue.h"
#include <ros/ros.h>
#include <unordered_set>
#include <sys/prctl.h>



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

    std::shared_ptr<sstd::BaseThread> recvFdThread{};
    sstd::atomic_queue<std::shared_ptr<std::function<void(SOCKET)>>> task_queue;

    template <asio::completion_token_for<void(SOCKET)> CompletionToken>
    auto async_get_socket(CompletionToken&& token);

    asio::io_context ctx{};

    std::unordered_set<RefConnectInstance> connectInstances{};

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


    this->logger.info("DefaultAgent::MAIN");

    co_spawn(ctx, listenFd(), asio::detached);

    ctx.run();
    exit(0);
}

void DefaultAgentManager::Impl::setSocketPipe(int pipe)
{
    pipeFd = pipe;
    recvFdThread =
            std::shared_ptr<sstd::BaseThread>(new sstd::thread([this]
                                                               {
                                                                   for (;;) {
                                                                       auto fd = recv_fd();
                                                                       auto task = task_queue.pop();
                                                                       (*task)(fd);
                                                                   }
                                                               }),
                                              // destroy thread
                                              [](sstd::BaseThread *th)
                                              {
                                                  if (th)
                                                      th->terminate();
                                                  delete th;
                                              });

}

template <asio::completion_token_for<void(SOCKET)> CompletionToken>
auto DefaultAgentManager::Impl::async_get_socket(CompletionToken&& token)
{

    // Define a function object that contains the code to launch the asynchronous
    // operation. This is passed the concrete completion handler, followed by any
    // additional arguments that were passed through the call to async_initiate.
    auto init = [this](asio::completion_handler_for<void(SOCKET)> auto handler){
        // According to the rules for asynchronous operations, we need to track
        // outstanding work against the handler's associated executor until the
        // asynchronous operation is complete.

        auto ref_handler = std::make_shared<decltype(handler)>(std::move(handler));

        auto work = asio::make_work_guard(*ref_handler);

        std::function<void(SOCKET)> callback = [
                work = std::move(work),
                ref_handler = std::move(ref_handler)
        ](SOCKET socketFd) mutable
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
                                                        socketFd
                                                ]() mutable
                                                {
                                                    std::move(*ref_handle)(socketFd);
                                                }));
        };

        task_queue.push(std::make_shared<std::function<void(SOCKET)>>(std::move(callback)));

    };

    // The async_initiate function is used to transform the supplied completion
    // token to the completion handler. When calling this function we explicitly
    // specify the completion signature of the operation. We must also return the
    // result of the call since the completion token may produce a return value,
    // such as a future.
    return asio::async_initiate<CompletionToken, void(SOCKET)>(
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

    int fd = *(int *) CMSG_DATA((cmsghdr *) &cmBuf);
    return fd;
}

awaitable<void> DefaultAgentManager::Impl::listenFd()
{
    for (;;) {
        try{

            auto fd = co_await async_get_socket(asio::use_awaitable);

            RefSocketor client = std::make_shared<tcp::socket>(co_await this_coro::executor, asio::ip::tcp::v4(), fd);
            logger.debug("recv a socketor, address: {}, port: {}", client->remote_endpoint().address().to_string(),
                         client->remote_endpoint().port());

            // TODO: create clientContext and push it into contextManager
            auto executor = client->get_executor();
            co_spawn(executor, [client = std::move(client), this]() -> awaitable<void>
            {
                auto refConnectInstance = std::make_shared<ConnectInstance>(client);
                connectInstances.insert(refConnectInstance);
                try {
                    co_await refConnectInstance->MAIN();
                }
                catch (std::exception &e) {
                    logger.error("ConnectInstance::MAIN error, reason: {}", e.what());
                }
                connectInstances.erase(refConnectInstance);
                co_return;
            }, asio::detached);
        }
        catch(std::exception &e){
            logger.error("catch exception in {}@{} : {}", __FILE__, __LINE__, e.what());
        }

    }

}


