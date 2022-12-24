//
// Created by HWZ on 2022/11/3.
//

#ifndef ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_JSON_H
#define ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_JSON_H

#include "../SDKException.h"
#include <ros/console.h>
using namespace std::string_literals;

#define _STR(x) _VAL(x)

#define _VAL(x) #x

#define RAPIDJSON_ASSERT(x) \
    if (!(x)) {     \
        auto fun = __func__; \
        auto str = "Rapidjson assert failed at: "  __FILE__ "@" _STR(__LINE__) "@"s + fun;\
        throw SDKException(str); \
    }

#define RAPIDJSON_ASSERT_THROWS 1

#ifndef RAPIDJSON_HAS_STDSTRING
#define RAPIDJSON_HAS_STDSTRING 1
#endif

#include "rapidjson/document.h"
#include "rapidjson/error/en.h"

class JsonParse : public rapidjson::Document
{
public:
    explicit JsonParse(std::string_view str) : rapidjson::Document()
    {
        Parse(str.data(), str.size());
    }
};

#endif //ROS_HYBIRD_SDK_SRC_ROS_HYBIRD_SDK_SERVER_SRC_JSON_H
