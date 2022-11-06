//
// Created by HWZ on 2022/11/3.
//
#include "DefaultAgent.h"
#include "Log.h"
#include <sstl/thread.h>
#include <sstl/atomic_queue.h>
#include "RefSocketor.h"

#include <asio.hpp>
#include <asio/experimental/as_tuple.hpp>
#include <asio/experimental/awaitable_operators.hpp>

using asio::awaitable;
using asio::buffer;
using asio::co_spawn;
using asio::ip::tcp;
namespace this_coro = asio::this_coro;
using namespace asio::experimental::awaitable_operators;
using std::chrono::steady_clock;
using namespace std::chrono_literals;
using namespace std::string_literals;

constexpr auto use_nothrow_awaitable = asio::experimental::as_tuple(asio::use_awaitable);


awaitable<void> timeout(steady_clock::duration duration)
{
    asio::steady_timer timer(co_await this_coro::executor);
    timer.expires_after(duration);
    co_await timer.async_wait(use_nothrow_awaitable);
}

struct DefaultAgent::Impl
{

    int recv_fd();

    Log logger{"DefaultAgent"};

    [[noreturn]] void MAIN();

    void setSocketPipe(int pipe);

    int pipeFd{};

    std::shared_ptr<sstd::BaseThread> recvFdThread{};

    sstd::atomic_queue<SOCKET> socketQueue{};

    awaitable<void> listenFd();

    asio::io_context ctx{};

};


[[noreturn]] void DefaultAgent::MAIN()
{
    implPtr->MAIN();
}

DefaultAgent::DefaultAgent() : implPtr(new DefaultAgent::Impl)
{
}

void DefaultAgent::setSocketPipe(int pipe)
{
    implPtr->setSocketPipe(pipe);
}

[[noreturn]]void DefaultAgent::Impl::MAIN()
{
    this->logger.info("DefaultAgent::MAIN");

    co_spawn(ctx, listenFd(), asio::detached);

    ctx.run();
    exit(0);
}

void DefaultAgent::Impl::setSocketPipe(int pipe)
{
    pipeFd = pipe;
    recvFdThread =
            std::shared_ptr<sstd::BaseThread>(new sstd::thread([this]
                                                               {
                                                                   for (;;) {
                                                                       auto socketFd = this->recv_fd();
                                                                       socketQueue.push(socketFd);
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

int DefaultAgent::Impl::recv_fd()
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

awaitable<void> DefaultAgent::Impl::listenFd()
{
    auto waitTime = 10ms;
    for (;;) {
        SOCKET fd;
        if (auto fdOpt = socketQueue.try_pop(); !fdOpt.has_value()) {
            co_await timeout(waitTime);
            waitTime = min(waitTime + 1ms, 1000ms);
            continue;
        } else
            fd = fdOpt.value();

        RefSocketor client = std::make_shared<tcp::socket>(co_await this_coro::executor, asio::ip::tcp::v4(), fd);
        logger.debug("recv a socketor, address: {}, port: {}", client->remote_endpoint().address().to_string(),
                     client->remote_endpoint().port());

        // TODO: create clientContext and push it into contextManager
        co_spawn(client->get_executor(), [client, this]() -> awaitable<void>
        {
            co_await client->async_send(buffer("in DefaultAgent, next operation not implement."s),
                                        use_nothrow_awaitable);
        }, asio::detached);

    }
}
