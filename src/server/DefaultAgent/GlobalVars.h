//
// Created by HWZ on 2023/1/14.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_DEFAULTAGENT_CALLBACKQUEUES_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_DEFAULTAGENT_CALLBACKQUEUES_H
#include <ros/callback_queue.h>
#include <asio/thread_pool.hpp>
#include <sstl/thread.h>
#include <memory>

inline auto g_topicThreadNums = [](){auto res = sstd::getCpuNums() / 6; return res == 0 ? 1 : res; }();
inline auto g_srvThreadNums = [](){auto res = sstd::getCpuNums() / 3; return res == 0 ? 1 : res; }();
inline auto g_coroThreadNums = [](){auto res = sstd::getCpuNums() / 8; return res == 0 ? 1 : res; }();

inline ros::CallbackQueue g_topicQueue{};
inline ros::CallbackQueue g_serviceQueue{};
inline std::unique_ptr<asio::thread_pool> g_serviceCallPool{};


#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_DEFAULTAGENT_CALLBACKQUEUES_H
