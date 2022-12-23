//
// Created by HWZ on 2022/12/17.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_ASIOHEADER_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_ASIOHEADER_H
#include <asio/ip/tcp.hpp>
#include <asio/experimental/as_tuple.hpp>
#include <asio/experimental/awaitable_operators.hpp>
#include <asio/detached.hpp>
#include <asio/read_until.hpp>
#include <asio/bind_allocator.hpp>
#include <asio/recycling_allocator.hpp>
using asio::awaitable;
using asio::buffer;
using asio::co_spawn;
using asio::ip::tcp;
namespace this_coro = asio::this_coro;
using namespace asio::experimental::awaitable_operators;
using std::chrono::steady_clock;
constexpr auto use_nothrow_awaitable = asio::experimental::as_tuple(asio::use_awaitable);
inline awaitable<void> timeout(steady_clock::duration duration)
{
    asio::steady_timer timer(co_await this_coro::executor);
    timer.expires_after(duration);
    co_await timer.async_wait(use_nothrow_awaitable);
}
#define HYBRID_DELIMITER "____delimiter____"
#define HYBRID_DELIMITER_SIZE (sizeof(HYBRID_DELIMITER) - 1)
#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_ASIOHEADER_H
