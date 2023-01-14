msgFileName = "/home/pi/MyMsg.msg"  # will be replaced
import re
import sys
import os
import time
from enum import IntEnum

msgName = re.search(R'(.*[/\\])?(\w+)\.msg', msgFileName).group(2)

class TypeTrail:
    def __init__(self, fieldType: int, builtinType: int, msgType: str, msgPackage: str, arraySize: int, constData: str,
                 varName: str):
        self.fieldType = fieldType
        self.builtinType = builtinType
        self.msgType = msgType
        self.msgPackage = msgPackage
        self.arraySize = arraySize
        self.constData = constData
        self.varName = varName

# will be replaced
msgVars = [
    TypeTrail(17, 2, '', '', 0, '1', 'DEBUG'),
    TypeTrail(17, 2, '', '', 0, '2', 'INFO'),
    TypeTrail(17, 2, '', '', 0, '4', 'WARN'),
    TypeTrail(17, 2, '', '', 0, '8', 'ERROR'),
    TypeTrail(17, 2, '', '', 0, '16', 'FATAL'),
    TypeTrail(2, 0, 'Header', 'std_msgs', 0, '', 'header'),
    TypeTrail(1, 12, '', '', 0, '', 'str'),
    TypeTrail(1, 2, '', '', 0, '', 'c'),
    TypeTrail(1, 4, '', '', 0, '', 's'),
    TypeTrail(1, 6, '', '', 0, '', 'i'),
    TypeTrail(1, 8, '', '', 0, '', 'l'),
    TypeTrail(1, 13, '', '', 0, '', 't'),
    TypeTrail(1, 14, '', '', 0, '', 'du'),
    TypeTrail(0x01 | 0x40, 6, '', '', 0, '', 'vi'),
    TypeTrail(0x01 | 0x20, 12, '', '', 5, '', 'strs'),
    TypeTrail(0x02 | 0x40, 0, 'Byte', 'std_msgs', 0, '', 'bytes'),
    TypeTrail(0x02 | 0x20, 0, 'Int32', 'std_msgs', 5, '', 'int5'),
    TypeTrail(0x02, 0, 'MultiArrayLayout', 'std_msgs', 0, '', 'layout'),
]


class FieldTypes(IntEnum):
    BuiltIn         = 0x01
    Msg             = 0x02
    Constexpr       = 0x10
    Array           = 0x20
    Vector          = 0x40

header = '''
//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by ros_hybrid_protoc on  {}
// Do not Edit!
//
// wrapping message: {}
'''.format(time.asctime(time.localtime(time.time())), msgName)

builtInTypeCppTypeMap = [
    "None",
    "uint8_t",
    "int8_t",
    "uint8_t",
    "int16_t",
    "uint16_t",
    "int32_t",
    "uint32_t",
    "int64_t",
    "uint64_t",
    "float",
    "double",
    "std::string",
    "ros::Time",
    "ros::Duration"
]

systemRes = os.system('rosmsg show {} > {}.tmp'.format(msgName, msgName))
if systemRes != 0:
    print('rosmsg show {} failed!'.format(msgName))
    sys.exit(1)

with open('{}.tmp'.format(msgName), 'r') as f:
    [rosNamespace, rosMsgType] = re.search(r'\[(.*)]:', f.readline()).group(1).split('/')
    rosMsgType = '::' + rosNamespace + '::' + rosMsgType
os.remove('{}.tmp'.format(msgName))
hybridMsgType = '::hybrid' + rosMsgType

defineStart = \
'''
#ifndef HYBRID_MSG_{0}_CPP
#define HYBRID_MSG_{0}_CPP
'''.format(msgName.upper())

defineEnd = \
'''
#endif // HYBRID_MSG_{}_CPP
'''.format(msgName.upper())

include = '#include "{}.pb.h"\n'.format(msgName)

include += \
'''
#include <memory>
#include <functional>
#include <ros/node_handle.h>
#include <interface.h>
#include <google/protobuf/util/json_util.h>
'''

include += '#include <{}/{}.h>\n'.format(rosNamespace, msgName)

for var in msgVars:
    if var.fieldType & FieldTypes.Msg:  # FieldTypes::Msg
        include += '#include <msgs/{}/{}.server.cpp>\n'.format(var.msgPackage, var.msgType)

include += 'using namespace std::string_literals;\n'

xxxCoverToProtoContent = ''
for var in msgVars:
    if var.fieldType & FieldTypes.Constexpr:
        continue
    if var.fieldType & FieldTypes.BuiltIn:
        if var.builtinType == 13 or var.builtinType == 14:  # Time or Duration
            if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
                xxxCoverToProtoContent += \
'''
    for (auto &data : rosMsg.{0}) {{
        auto *p = protoMsg.add_{0}();
        p->set_seconds(data.sec);
        p->set_nanos(static_cast<int32_t>(data.nsec));    
'''.format(var.varName)
                continue
            else:
                xxxCoverToProtoContent += \
'''
    protoMsg.mutable_{0}()->set_seconds(rosMsg.{0}.sec);
    protoMsg.mutable_{0}()->set_nanos(static_cast<int32_t>(rosMsg.{0}.nsec));
'''.format(var.varName)
                continue
        if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
            xxxCoverToProtoContent += '    for (auto &data : rosMsg.{0}) protoMsg.add_{0}(data); \n'.format(var.varName)
        else:
            xxxCoverToProtoContent += '    protoMsg.set_{0}(rosMsg.{0}); \n'.format(var.varName)
    elif var.fieldType & FieldTypes.Msg:
        if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
            xxxCoverToProtoContent += '    for (auto &data : rosMsg.{0}) *protoMsg.add_{0}() = {1}CoverToProto(data); \n'.format(
                var.varName, var.msgType)
        else:
            xxxCoverToProtoContent += '    *protoMsg.mutable_{0}() = {1}CoverToProto(rosMsg.{0}); \n'\
                .format(var.varName, var.msgType)


xxxCoverToProtoStartEnd = \
'''
{0} {1}CoverToProto(const {2} &rosMsg)
{{
    {0} protoMsg;
{3}
    return protoMsg;
}}
'''.format(hybridMsgType, msgName, rosMsgType, xxxCoverToProtoContent)

xxxCoverToRosContent = ''
for var in msgVars:
    if var.fieldType & FieldTypes.Constexpr:
        continue
    if var.fieldType & FieldTypes.BuiltIn:
        if var.builtinType == 13 or var.builtinType == 14:  # Time or Duration
            if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
                xxxCoverToRosContent += \
'''
    for (auto &data : protoMsg.{0}())
        rosMsg.{0}.emplace_back(ros::Time(data.seconds(), data.nanos()));
'''.format(var.varName)
                continue
            else:
                xxxCoverToRosContent += \
'''
    rosMsg.{0}.sec = protoMsg.{0}().seconds();
    rosMsg.{0}.nsec = protoMsg.{0}().nanos();
'''.format(var.varName)
                continue
        if var.fieldType & FieldTypes.Array:
            xxxCoverToRosContent += \
'''
    if (protoMsg.{0}_size() != {1}::_{0}_type::size())
        throw std::runtime_error("size of {0} is not match!");
    for (size_t i = 0; i < {1}::_{0}_type::size(); ++i)
        rosMsg.{0}[i] = protoMsg.{0}(static_cast<int>(i));    
'''.format(var.varName, rosMsgType)
        elif var.fieldType & FieldTypes.Vector:
            xxxCoverToRosContent += '    std::copy(protoMsg.{0}().begin(), protoMsg.{0}().end(), std::back_inserter(rosMsg.{0}));\n'\
                .format(var.varName)
        else:
            xxxCoverToRosContent += '    rosMsg.{0} = protoMsg.{0}(); \n'.format(var.varName)
    else:
        if var.fieldType & FieldTypes.Array:
            xxxCoverToRosContent += \
'''
    if (protoMsg.{0}_size() != {1}::_{0}_type::size())
        throw std::runtime_error("size of {0} is not match!");
    for (size_t i = 0; i < {1}::_{0}_type::size(); ++i)
        rosMsg.{0}[i] = {2}CoverToRos(protoMsg.{0}(static_cast<int>(i)));
'''.format(var.varName, rosMsgType, var.msgType)
        elif var.fieldType & FieldTypes.Vector:
            xxxCoverToRosContent += \
'''
    rosMsg.{0}.reserve(protoMsg.{0}_size());
    for (const auto &data : protoMsg.{0}())
        rosMsg.{0}.emplace_back({1}CoverToRos(data));
'''.format(var.varName, var.msgType)
        else:
            xxxCoverToRosContent += '    rosMsg.{0} = {1}CoverToRos(protoMsg.{0}()); \n'\
                .format(var.varName, var.msgType)


xxxCoverToRosStartEnd = \
'''
{0} {1}CoverToRos(const {2} &protoMsg)
{{
    {0} rosMsg;
{3}
    return rosMsg;
}}
'''.format(rosMsgType, msgName, hybridMsgType, xxxCoverToRosContent)


Build_xxx_SHARED_LIB_defineStart = '#ifdef BUILD_{}_SHARED_LIB\n'.format(msgName.upper())

namespaceStart = 'namespace hybrid\n{'
namespaceEnd = '} // namespace hybrid'

Build_xxx_SHARED_LIB_defineEnd = '#endif // BUILD_{}_SHARED_LIB\n'.format(msgName.upper())

classMsgPublisher = \
'''
class {0}Publisher : public MsgPublisher
{{
public:
    {0}Publisher(const std::string &topic, uint32_t queue_size, ros::CallbackQueue *callbackQueue, bool is_protobuf, bool latch = false) : MsgPublisher(topic,
                                                                                                  queue_size,
                                                                                                  callbackQueue,
                                                                                                  is_protobuf,
                                                                                                  latch),
                                                                                                  is_protobuf(is_protobuf)
    {{
        if (callbackQueue)    
            nh.setCallbackQueue(callbackQueue);    
        pub = nh.advertise<{3}>(topic, queue_size, latch);
    }}

    void publish(const std::string &msgBuf) override
    {{
        {1}  protoMsg;
        if (is_protobuf) {{
            if (!protoMsg.ParseFromString(msgBuf))
                throw std::runtime_error(__func__ + "msgBuf parse fail!"s);
        }} else {{
            auto state = google::protobuf::util::JsonStringToMessage(msgBuf, &protoMsg);
            if (!state.ok())
                throw std::runtime_error(__func__ + "msgBuf parse fail: "s + state.ToString());
        }}
        pub.publish({2}CoverToRos(protoMsg));
    }}
    
    ~{0}Publisher() override = default;
    private:
        ros::NodeHandle nh{{}};
        ros::Publisher pub{{}};
        bool is_protobuf{{}};
}};
'''.format(msgName, hybridMsgType, msgName, rosMsgType)

classMsgSubscriber = \
'''
class {0}Subscriber : public MsgSubscriber
{{
public:
    {0}Subscriber(const std::string &topic, uint32_t queue_size, ros::CallbackQueue *callbackQueue, bool is_protobuf, const std::function<void(std::string)> &callback)
                : MsgSubscriber(topic,
                                queue_size,
                                callbackQueue,
                                is_protobuf,
                                callback)
    {{
        if (callbackQueue)
            nh.setCallbackQueue(callbackQueue);
        sub = nh.subscribe(topic, queue_size,
                               boost::function<void(const {1}::ConstPtr &ros_msg)>(
                                       [callback, is_protobuf](const {1}::ConstPtr &ros_msg)
                                       {{
                                           auto protoMsg = {0}CoverToProto(*ros_msg);
                                           if (is_protobuf)
                                               callback({0}CoverToProto(*ros_msg).SerializeAsString());
                                           else {{
                                               std::string jsonStr;
                                               auto state = google::protobuf::util::MessageToJsonString(protoMsg, &jsonStr);
                                               if (!state.ok())
                                                    throw std::runtime_error(__func__ + "msgBuf parse fail: "s + state.ToString());
                                               callback(jsonStr);
                                           }}
                                               
                                       }}
                               )
       );
    }}
    
    ~{0}Subscriber() override = default;
private:
    ros::NodeHandle nh{{}};
    ros::Subscriber sub{{}};
}};
'''.format(msgName, rosMsgType)

externCInterface = \
'''
extern "C" {{
hybrid::MsgPublisher *make_publisher(const std::string &topic, uint32_t queue_size, ros::CallbackQueue *callbackQueue, bool is_protobuf, bool latch)
{{
    return new hybrid::{0}Publisher(topic, queue_size, callbackQueue, is_protobuf, latch);
}}
hybrid::MsgSubscriber *make_subscriber(const std::string &topic, uint32_t queue_size, ros::CallbackQueue *callbackQueue, bool is_protobuf, const std::function<void(std::string)> &callback)
{{
    return new hybrid::{0}Subscriber(topic, queue_size, callbackQueue, is_protobuf, callback);
}}
}}
'''.format(msgName)

# output file
xxx_server_cpp = header + defineStart + include + xxxCoverToProtoStartEnd + xxxCoverToRosStartEnd + \
                 Build_xxx_SHARED_LIB_defineStart + namespaceStart + classMsgPublisher + classMsgSubscriber + \
                 namespaceEnd + externCInterface + Build_xxx_SHARED_LIB_defineEnd + defineEnd

with open('{}.server.cpp'.format(msgName), 'w') as f:
    f.write(xxx_server_cpp)

with open('result.txt', 'w') as f:
    f.write('msgs/' + rosNamespace)
    f.write('\n')
    f.write('{}.server.cpp'.format(msgName))

