//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_MSGGENERATOR_H
#define ROS_HYBRID_SDK_MSGGENERATOR_H

#include <vector>
#include "../Parser/typedef.h"

using namespace std::string_literals;

struct GenMsgServerUseOnlyResult
{
    std::string path;
    struct FileDescription
    {
        std::string name;
        std::string content;
    };
    std::vector<FileDescription> files;

};


inline GenMsgServerUseOnlyResult GenMsgServerUseOnly(const std::string &msgFileName, const std::vector<TypeTrail> &vars)
{
    std::string msgName;
    std::regex reg(R"((.*[\/\\])?(\w+)\.msg$)");
    msgName = std::regex_replace(msgFileName, reg, "$2");


    std::string header =
R"(//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on )" __DATE__ "  " __TIME__ R"(
// Do not Edit!
//
// wrapping message: )"s + msgName + R"(

)"s;


    /****************
     * xxx.h
     ****************/

    std::string pragma_once = "#pragma once\n"s;

    // include file
    std::string include;
    include += "#include <ros/node_handle.h>\n";
    bool hasRosHeader{false}, hasVector{false}, hasArray{false};
    for (const auto &var: vars) {
        switch (var.fieldType) {
            case FieldTypes::Header:
                hasRosHeader = true;
                break;
            case FieldTypes::Vector:
                hasVector = true;
                break;
            case FieldTypes::Array:
                hasArray = true;
                break;
            default:
                break;
        }
    }

    if (hasRosHeader)
        include += R"(#include "../MsgHeader.h")" "\n";

    if (hasArray)
        include += R"(#include <array>)" "\n";

    if (hasVector)
        include += R"(#include <vector>)" "\n";

    std::string namespaceStart =
R"(
namespace hybrid {
)";



    std::string namespaceEnd =
R"(
}
)";

    std::string structStart =
R"(
    struct )"s + msgName + R"(
    {
)";



    std::string structEnd =
R"(
        };
)";

    std::string structConstVar;
    std::string structVar;

    for (const auto &var: vars) {
        if (var.fieldType == FieldTypes::Header)
            structVar += R"(        Header )"s + var.name + "{};\n";
        else if (var.fieldType == FieldTypes::Constexpr)
            structConstVar += R"(        static const )"s + BuiltInTypeCppTypeMap[var.builtInType] + " "s + var.name + " = "s +
                              var.constData + ";\n";
        else if (var.fieldType == FieldTypes::Vector)
            structVar +=
                    R"(        std::vector<)"s + BuiltInTypeCppTypeMap[var.builtInType] + "> "s + var.name + "{};\n";
        else if (var.fieldType == FieldTypes::Array)
            structVar += R"(        std::array<)"s + BuiltInTypeCppTypeMap[var.builtInType] + ", "s +
                         std::to_string(var.arraySize) + "> "s + var.name + "{};\n";
        else if (var.fieldType == FieldTypes::BuiltIn)
            structVar += R"(        )"s + BuiltInTypeCppTypeMap[var.builtInType] + " "s + var.name + "{};\n";
        else
            throw std::runtime_error(__FILE__ ":"s + std::to_string(__LINE__) + " : " " unknown field type: " +
                                     std::to_string((int) var.fieldType));
    }

    std::string classPublisher =
R"(
    class )"s + msgName + R"(Publisher{
    public:
        )"s + msgName + R"(Publisher(const std::string &topic, uint32_t queue_size, bool latch = false);
        void publish(const )"s + msgName + R"( &msg);

        ros::Publisher pub;
    private:
        ros::NodeHandle nh;
    };
)";

    std::string classSubscriber =
R"(
    class )"s + msgName + R"(Subscriber{
    public:
        )"s + msgName + R"(Subscriber(const std::string &topic, uint32_t queue_size, const std::function<void()" +
                                  msgName + R"()> &callback );

        ros::Subscriber sub;
    private:
        ros::NodeHandle nh;
    };
)";


    std::string xxx_h =
            header + pragma_once + include + namespaceStart + structStart + structConstVar + "\n\n\n" + structVar +
            structEnd + classPublisher + classSubscriber + namespaceEnd;

    /****************
    * xxx.cpp
    ****************/
    include = R"(#include ")"s + msgName + R"(.h")" "\n";

    //get ros msg include path, use system call 'rosmsg show'
    std::string hybridMsgType = "hybrid::"s + msgName;
    std::string rosMsgType;
    {
        auto res = system(("rosmsg show " + msgName + " > temp.txt").c_str());
        if (res != 0)
            throw std::runtime_error(
                    __FILE__ ":"s + std::to_string(__LINE__) + " : " "exec rosmsg show " + msgName + " failed");
        std::ifstream ifs("temp.txt");
        if (!ifs.is_open())
            throw std::runtime_error("open temp.txt failed");

        std::string line;
        std::getline(ifs, line);
        ifs.close();
        // delete temp
        std::remove("temp.txt");

        reg = (R"(\[(.*)\]:)");
        std::smatch sm;
        std::regex_match(line, sm, reg);
        if (sm.size() < 2)
            throw std::runtime_error(
                    __FILE__ ":"s + std::to_string(__LINE__) + " : " "parse rosmsg show " + msgName + " failed");

        include += "#include <" + sm[1].str() + ".h>\n";

        std::string ros_namespace = sm[1].str().substr(0, sm[1].str().find('/'));
        rosMsgType = ros_namespace + "::" + msgName;
    }

    include += "\n\n";
    std::string PublisherConstructor =
hybridMsgType + R"(Publisher::)"s + msgName + R"(Publisher(const std::string &topic, uint32_t queue_size, bool latch)
{
    pub = nh.advertise<)"s + rosMsgType + R"(>(topic, queue_size, latch);
}

)";

    std::string PublisherPublishStart =
R"(
void )"s + hybridMsgType + R"(Publisher::publish(const )"s + hybridMsgType + R"( &msg)
{
)";

    std::string PublisherPublishEnd =
R"(
    pub.publish(ros_msg);
}

)";

    std::string PublisherPublishContent;
    PublisherPublishContent += "    " + rosMsgType + " ros_msg;\n";
    for (const auto &var: vars) {
        if (var.fieldType == FieldTypes::Header) {
            PublisherPublishContent +=
R"(
    ros_msg.header.stamp = msg.header.stamp;
    ros_msg.header.seq = msg.header.seq;
    ros_msg.header.frame_id = msg.header.frame_id;
)";
        }
        else if (var.fieldType == FieldTypes::Constexpr){
            continue;
        }
        else{
            PublisherPublishContent += "    ros_msg." + var.name + " = msg." + var.name + ";\n";
        }

    }

    std::string SubscriberConstructorStart =
hybridMsgType + R"(Subscriber::)"s + msgName + R"(Subscriber(const std::string &topic, uint32_t queue_size, const std::function<void()" +
                                               hybridMsgType + R"()> &callback )
{
    sub = nh.subscribe(topic,queue_size,
                     boost::function<void(
                             const )" + rosMsgType + R"(::ConstPtr &)>(
                             [callback](
                                     const )" + rosMsgType + R"(::ConstPtr &ros_msg)
                             {
)";
                                // .....
    std::string SubscriberConstructorEnd =
R"(
                                callback(std::move(msg));
                             }));
}

)";

    std::string SubscribeConstructorContext;
    SubscribeConstructorContext += "                                " + hybridMsgType + " msg;\n";
    for (const auto &var: vars) {
        if (var.fieldType == FieldTypes::Header) {
            SubscribeConstructorContext +=
R"(
                                msg.header.stamp = ros_msg->header.stamp;
                                msg.header.seq = ros_msg->header.seq;
                                msg.header.frame_id = ros_msg->header.frame_id;
)";
        }
        else if (var.fieldType == FieldTypes::Constexpr){
            continue;
        }
        else{
            SubscribeConstructorContext +=
"                                msg." + var.name + " = ros_msg->" + var.name + ";\n";
        }
    }

    std::string xxx_cpp =
            header + include + PublisherConstructor + PublisherPublishStart + PublisherPublishContent +
            PublisherPublishEnd + SubscriberConstructorStart + SubscribeConstructorContext + SubscriberConstructorEnd;

    /*********************
     * CMakeLists.txt
     ********************/


    std::string  CMakeLists_txt =
R"(# generated automatically by messageGenerator on )" __DATE__ "  " __TIME__ R"(
# Do not Edit!
# use it by add_subdirectory()
set(WRAPPER_NAME wrapper)"s + msgName + R"()
add_library(${WRAPPER_NAME} STATIC )"s + msgName + R"(.cpp)
target_link_libraries(${WRAPPER_NAME} ${catkin_LIBRARIES})
)";

    GenMsgServerUseOnlyResult result;
    result.path = msgName;
    result.files.push_back({msgName + ".h", xxx_h});
    result.files.push_back({msgName + ".cpp", xxx_cpp});
    result.files.push_back({"CMakeLists.txt", CMakeLists_txt});

    return result;
}

#endif //ROS_HYBRID_SDK_MSGGENERATOR_H
