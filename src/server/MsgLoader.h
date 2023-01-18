//
// Created by HWZ on 2022/12/18.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_MESSAGELOADER_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_MESSAGELOADER_H

#include "Interface.h"
#include <string>
#include <dlfcn.h>
#include <unordered_map>
#include "../SDKException.h"

// Think: Do dynamically loaded libraries need to be unloaded?
class MsgLoader
{
public:
    static auto getPublisher(const std::string &name)
    {
        if (dlHandleMap.count(name) == 0) {
            auto dllName = "lib" + name + ".so";
            auto res = dlopen(dllName.c_str(), RTLD_LAZY);
            if (!res)
                throw SDKException("dlopen error: " + std::string(dlerror()));
            dlHandleMap[name] = res;
        }
        if (pubFuncMap.count(name) == 0) {
            auto pVoid = dlsym(dlHandleMap[name], "make_publisher");
            if (!pVoid)
                throw SDKException("dlsym error: " + std::string(dlerror()));
            auto func = reinterpret_cast<hybrid::MsgPublisher *(*)(const std::string &, uint32_t, ros::CallbackQueue*, bool, bool)>(pVoid);
            pubFuncMap[name] = func;
        }
        return pubFuncMap[name];
    }

    static auto getSubscriber(const std::string &name)
    {
        if (dlHandleMap.count(name) == 0) {
            auto dllName = "lib" + name + ".so";
            auto res = dlopen(dllName.c_str(), RTLD_LAZY);
            if (!res)
                throw SDKException("dlopen error: " + std::string(dlerror()));
            dlHandleMap[name] = res;
        }
        if (subFuncMap.count(name) == 0) {
            auto pVoid = dlsym(dlHandleMap[name], "make_subscriber");
            if (!pVoid)
                throw SDKException("dlsym error: " + std::string(dlerror()));
            auto func = reinterpret_cast<hybrid::MsgSubscriber *(*)(const std::string &,
                                                                    uint32_t,
                                                                    ros::CallbackQueue*,
                                                                    bool,
                                                                    const std::function<void(std::string)> &)>(pVoid);
            subFuncMap[name] = func;
        }
        return subFuncMap[name];
    }

    static auto getSeriviceServer(const std::string &name)
    {
        if (dlHandleMap.count(name) == 0) {
            auto dllName = "lib" + name + ".so";
            auto res = dlopen(dllName.c_str(), RTLD_LAZY);
            if (!res)
                throw SDKException("dlopen error: " + std::string(dlerror()));
            dlHandleMap[name] = res;
        }
        if (serviceServerFuncMap.count(name) == 0) {
            auto pVoid = dlsym(dlHandleMap[name], "make_service_server");
            if (!pVoid)
                throw SDKException("dlsym error: " + std::string(dlerror()));
            auto func = reinterpret_cast<hybrid::SrvAdvertiser *(*)(const std::string &,
                                                                    ros::CallbackQueue*,
                                                                    bool,
                                                                    const std::function<std::string(std::string)> &)>(pVoid);
            serviceServerFuncMap[name] = func;
        }
        return serviceServerFuncMap[name];
    }

private:
    inline static std::unordered_map<std::string, void *> dlHandleMap{};
    inline static std::unordered_map<std::string, hybrid::MsgPublisher *(*)(const std::string &, uint32_t, ros::CallbackQueue*, bool, bool)>
        pubFuncMap{};
    inline static std::unordered_map<std::string,
                                     hybrid::MsgSubscriber *(*)(const std::string &,
                                                                uint32_t,
                                                                ros::CallbackQueue*,
                                                                bool,
                                                                const std::function<void(std::string)> &)> subFuncMap{};

    inline static std::unordered_map<std::string, hybrid::SrvAdvertiser *(*)(const std::string &,
                                                                             ros::CallbackQueue*,
                                                                             bool,
                                                                             const std::function<std::string(std::string)> &)> serviceServerFuncMap{};

};

#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_SERVER_MESSAGELOADER_H
