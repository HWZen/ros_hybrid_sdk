//
// Created by HWZ on 2023/1/14.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_HYBRIDSPINNER_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_HYBRIDSPINNER_H

#include <ros/callback_queue.h>
#include <csignal>
#include <sstl/thread.h>
#include <ros/node_handle.h>
#include "Log.h"

class HybridSpinner
{
public:
    HybridSpinner() = default;
    explicit HybridSpinner(std::string_view name, size_t thread_num = sstd::getCpuNums()) :
        logger(name, LogFlag::CONSOLE_LOGGER),
        thread_num(thread_num) {}
    void spin(ros::CallbackQueue *queue)
    {
        if (thread_num == 0)
        {
            logger.warn("thread_num is 0, spin will start one thread");
            thread_num = 1;
        }
        logger.info("start spin, thread_num: {}", thread_num);
        auto threadFunc = [&, queue](){
            // disable all signals in this thread
            sigset_t set{};
            sigfillset(&set);
            pthread_sigmask(SIG_SETMASK, &set, nullptr);

            ros::WallDuration timeout(0.1);
            ros::NodeHandle nd;
            while ( nd.ok() && !stop_flag) {
                try {
                    queue->callOne(timeout);
                }
                catch (const std::exception &e) {
                    logger.error("catch exception: {}", e.what());
                }
            }
        };

        for (size_t i = 0; i < thread_num; ++i)
            threads.emplace_back(sstd::thread(threadFunc));

    }

    void stop()
    {
        stop_flag = true;
        for (auto &thread : threads)
            thread.join();
        threads.clear();
    }

    ~HybridSpinner()
    {
        stop();
    }

private:
    size_t thread_num{};
    Log logger{"HybridSpinner", LogFlag::CONSOLE_LOGGER};
    volatile bool stop_flag{false};
    std::vector<sstd::any_thread> threads;
};



#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_HYBRIDSPINNER_H
