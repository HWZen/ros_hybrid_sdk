srvFileName = "home/pi/MyService.srv"  # will be replaced
import re
import sys
import os
import time
from enum import IntEnum

srvName = re.search(R'(.*[/\\])?(\w+)\.srv', srvFileName).group(2)


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
requestVars = [
    TypeTrail(2, 0, 'MyMsg', 'ros_hybrid_sdk', 0, '', 'myMsg'),

]
responseVars = [
    TypeTrail(2, 0, 'Header', 'std_msgs', 0, '', 'header'),
]

srvVars = requestVars + responseVars


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
'''.format(time.asctime(time.localtime(time.time())), srvName)

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

systemRes = os.system('rossrv show {} > {}.tmp'.format(srvName, srvName))
if systemRes != 0:
    print('rossrv show {} failed!'.format(srvName))
    sys.exit(1)

with open('{}.tmp'.format(srvName), 'r') as f:
    [rosNamespace, rosMsgType] = re.search(r'\[(.*)]:', f.readline()).group(1).split('/')
    rosMsgType = '::' + rosNamespace + '::' + rosMsgType
os.remove('{}.tmp'.format(srvName))
hybridMsgType = '::hybrid' + rosMsgType

defineStart = \
'''
#ifndef HYBRID_SRV_{0}_CPP
#define HYBRID_SRV_{0}_CPP
'''.format(srvName.upper())

defineEnd = \
'''
#endif // HYBRID_SRV_{}_CPP
'''.format(srvName.upper())

include = '#include "{}.pb.h"\n'.format(srvName)

include += \
'''
#include <ros/node_handle.h>
#include <interface.h>
#include <google/protobuf/util/json_util.h>
'''

include += '#include <{}/{}.h>\n'.format(rosNamespace, srvName)

for var in srvVars:
    if var.fieldType & FieldTypes.Msg:
        include += '#include <msgs/{}/{}.server.cpp>\n'.format(var.msgPackage, var.msgType)

include += 'using namespace std::string_literals;\n'

xxxRequestCoverToProtoDeclare = \
'''
{0}::Request {1}RequestCoverToProto(const {2}::Request &rosMsg);
'''.format(hybridMsgType, srvName, rosMsgType)

xxxResponseCoverToProtoDeclare = \
'''
{0}::Response {1}ResponseCoverToProto(const {2}::Response &rosMsg);
'''.format(hybridMsgType, srvName, rosMsgType)

xxxRequestCoverToRosDeclare = \
'''
{2}::Request {1}RequestCoverToRos(const {0}::Request &hybridMsg);
'''.format(hybridMsgType, srvName, rosMsgType)

xxxResponseCoverToRosDeclare = \
'''
{2}::Response {1}ResponseCoverToRos(const {0}::Response &hybridMsg);
'''.format(hybridMsgType, srvName, rosMsgType)

xxxCoverToProtoDeclare = \
'''
{0} {1}CoverToProto(const {2} &rosMsg);
'''.format(hybridMsgType, srvName, rosMsgType)

xxxCoverToRosDeclare = \
'''
{2} {1}CoverToRos(const {0} &hybridMsg);
'''.format(hybridMsgType, srvName, rosMsgType)


def msgsCoverToProtoContent(vars):
    res = ''
    for var in vars:
        if var.fieldType & FieldTypes.Constexpr:
            continue
        if var.fieldType & FieldTypes.BuiltIn:
            if var.builtinType == 13 or var.builtinType == 14:  # Time or Duration
                if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
                    res += \
'''
    for (auto &data : rosMsg.{0}) {{
        auto *p = protoMsg.add_{1}();
        p->set_seconds(data.sec);
        p->set_nanos(static_cast<int32_t>(data.nsec));
    }}
'''.format(var.varName, var.varName.lower())
                    continue
                else:
                    res += \
'''
    protoMsg.mutable_{1}()->set_seconds(rosMsg.{0}.sec);
    protoMsg.mutable_{1}()->set_nanos(static_cast<int32_t>(rosMsg.{0}.nsec));
'''.format(var.varName, var.varName.lower())
                    continue
            if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
                res += '    for (auto &data : rosMsg.{0}) protoMsg.add_{1}(data); \n'.format(var.varName, var.varName.lower())
            else:
                res += '    protoMsg.set_{1}(rosMsg.{0}); \n'.format(var.varName, var.varName.lower())
        elif var.fieldType & FieldTypes.Msg:
            if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
                res += '    for (auto &data : rosMsg.{0}) *protoMsg.add_{2}() = {1}CoverToProto(data); \n'.format(
                    var.varName, var.msgType, var.varName.lower())
            else:
                res += '    *protoMsg.mutable_{2}() = {1}CoverToProto(rosMsg.{0}); \n' \
                    .format(var.varName, var.msgType, var.varName.lower())
    return res


def msgsCoverToRosContent(vars):
    res = ''
    for var in vars:
        if var.fieldType & FieldTypes.Constexpr:
            continue
        if var.fieldType & FieldTypes.BuiltIn:
            if var.builtinType == 13 or var.builtinType == 14:  # Time or Duration
                if var.fieldType & FieldTypes.Array or var.fieldType & FieldTypes.Vector:
                    res += \
'''
    for (auto &data : protoMsg.{1}())
        rosMsg.{0}.emplace_back(ros::Time(data.seconds(), data.nanos()));
'''.format(var.varName, var.varName.lower())
                    continue
                else:
                    res += \
'''
    rosMsg.{0}.sec = protoMsg.{1}().seconds();
    rosMsg.{0}.nsec = protoMsg.{1}().nanos();
'''.format(var.varName, var.varName.lower())
                    continue
            if var.fieldType & FieldTypes.Array:
                res += \
'''
    if (protoMsg.{2}_size() != {1}::_{0}_type::size())
        throw std::runtime_error("size of {0} is not match!");
    for (size_t i = 0; i < {1}::_{0}_type::size(); ++i)
        rosMsg.{0}[i] = protoMsg.{2}(static_cast<int>(i));    
'''.format(var.varName, rosMsgType, var.varName.lower())
            elif var.fieldType & FieldTypes.Vector:
                res += '    std::copy(protoMsg.{1}().begin(), protoMsg.{1}().end(), std::back_inserter(rosMsg.{0}));\n' \
                    .format(var.varName, var.varName.lower())
            else:
                res += '    rosMsg.{0} = protoMsg.{1}(); \n'.format(var.varName, var.varName.lower())
        else:
            if var.fieldType & FieldTypes.Array:
                res += \
'''
    if (protoMsg.{3}_size() != {1}::_{0}_type::size())
        throw std::runtime_error("size of {0} is not match!");
    for (size_t i = 0; i < {1}::_{0}_type::size(); ++i)
        rosMsg.{0}[i] = {2}CoverToRos(protoMsg.{3}(static_cast<int>(i)));
'''.format(var.varName, rosMsgType, var.msgType, var.varName.lower())
            elif var.fieldType & FieldTypes.Vector:
                res += \
'''
    rosMsg.{0}.reserve(protoMsg.{2}_size());
    for (const auto &data : protoMsg.{2}())
        rosMsg.{0}.emplace_back({1}CoverToRos(data));
'''.format(var.varName, var.msgType, var.varName.lower())
            else:
                res += '    rosMsg.{0} = {1}CoverToRos(protoMsg.{2}()); \n' \
                    .format(var.varName, var.msgType, var.varName.lower())
    return res


xxxRequestCoverToProtoContent = msgsCoverToProtoContent(requestVars)
xxxRequestCoverToRosContent = msgsCoverToRosContent(requestVars)
xxxResponseCoverToProtoContent = msgsCoverToProtoContent(responseVars)
xxxResponseCoverToRosContent = msgsCoverToRosContent(responseVars)

xxxRequestCoverToProto = \
'''
{0}::Request {1}RequestCoverToProto(const {2}::Request &rosMsg)
{{
    {0}::Request protoMsg;
{3}
    return protoMsg;
}}
'''.format(hybridMsgType, srvName, rosMsgType, xxxRequestCoverToProtoContent)

xxxRequestCoverToRos = \
'''
{2}::Request {1}RequestCoverToRos(const {0}::Request &protoMsg)
{{
    {2}::Request rosMsg;
{3}
    return rosMsg;
}}
'''.format(hybridMsgType, srvName, rosMsgType, xxxRequestCoverToRosContent)

xxxResponseCoverToProto = \
'''
{0}::Response {1}ResponseCoverToProto(const {2}::Response &rosMsg)
{{
    {0}::Response protoMsg;
{3}
    return protoMsg;
}}
'''.format(hybridMsgType, srvName, rosMsgType, xxxResponseCoverToProtoContent)

xxxResponseCoverToRos = \
'''
{2}::Response {1}ResponseCoverToRos(const {0}::Response &protoMsg)
{{
    {2}::Response rosMsg;
{3}
    return rosMsg;
}}
'''.format(hybridMsgType, srvName, rosMsgType, xxxResponseCoverToRosContent)

xxxCoverToProto = \
'''
{0} {1}CoverToProto(const {2} &rosMsg)
{{
    {0} protoMsg;
    *protoMsg.mutable_request() = {1}RequestCoverToProto(rosMsg.request);
    *protoMsg.mutable_response() = {1}ResponseCoverToProto(rosMsg.response);
    return protoMsg;
}}
'''.format(hybridMsgType, srvName, rosMsgType)

xxxCoverToRos = \
'''
{2} {1}CoverToRos(const {0} &protoMsg)
{{
    {2} rosMsg;
    rosMsg.request = {1}RequestCoverToRos(protoMsg.request());
    rosMsg.response = {1}ResponseCoverToRos(protoMsg.response());
    return rosMsg;
}}
'''.format(hybridMsgType, srvName, rosMsgType)

Build_xxx_SHARED_LIB_defineStart = '#ifdef BUILD_{0}_SHARED_LIB\n'.format(srvName.upper())

namespaceStart = 'namespace hybrid\n{'
namespaceEnd = '} // namespace hybrid'

Build_xxx_SHARED_LIB_defineEnd = '#endif // BUILD_{0}_SHARED_LIB\n'.format(srvName.upper())

classSrvCaller = \
'''
class {0}Caller : public SrvCaller
{{
public:
    {0}Caller(const std::string &serviceName, ros::CallbackQueue* callbackQueue, bool is_protobuf):
                                                       SrvCaller(serviceName, callbackQueue, is_protobuf),
                                                       is_protobuf(is_protobuf){{
        if (callbackQueue)
            nh.setCallbackQueue(callbackQueue);
        client = nh.serviceClient<{2}>(serviceName);
    }}
    std::string call(const std::string &reqStr) override
    {{
        {1}::Request request;
        if (is_protobuf) {{
            if (!request.ParseFromString(reqStr))
                throw std::runtime_error("{0} parse request failed!");
        }} else {{
            auto status = google::protobuf::util::JsonStringToMessage(reqStr, &request);
            if (!status.ok())
                throw std::runtime_error("{0} parse request failed: " + status.ToString());
        }}
        {2} rosMsg;
        rosMsg.request = {0}RequestCoverToRos(request);
        if (!client.call(rosMsg))
           return {{}};
        {1}::Response response = {0}ResponseCoverToProto(rosMsg.response);
        if (is_protobuf)
            return response.SerializeAsString();
        else {{
            std::string resStr;
            auto status = google::protobuf::util::MessageToJsonString(response, &resStr);
            if (!status.ok())
                throw std::runtime_error("{0} parse response failed: " + status.ToString());
            return resStr;
        }}
    }}
    ~{0}Caller() override = default;
private:
    ros::NodeHandle nh{{}};
    ros::ServiceClient client{{}};
    bool is_protobuf;
}};
'''.format(srvName, hybridMsgType, rosMsgType)

classSrvAdvertiser = \
'''
class {0}Advertiser : public SrvAdvertiser
{{
public:
    {0}Advertiser(const std::string &serviceName,
                  ros::CallbackQueue* callbackQueue,
                  bool is_protobuf,
                  const std::function<std::string(std::string)> &callback):
                    SrvAdvertiser(serviceName, callbackQueue, is_protobuf, callback) {{
        if (callbackQueue)
            nh.setCallbackQueue(callbackQueue);
        server = nh.advertiseService(serviceName, boost::function<bool({2}::Request &,
                                                                       {2}::Response &)>(
                                                                       [=]({2}::Request &req,
                                                                          {2}::Response &res)->bool{{
            {1}::Request protoMsg = {0}RequestCoverToProto(req);
            
            {1}::Response protoRes;
            if (is_protobuf) {{
                std::string resBuf = callback(protoMsg.SerializeAsString());
                if (!protoRes.ParseFromString(resBuf))
                    throw std::runtime_error("{0} parse response failed!");
            }} else {{
                std::string jsonStr;
                auto status = google::protobuf::util::MessageToJsonString(protoMsg, &jsonStr);
                if (!status.ok())
                    throw std::runtime_error("{0} parse request failed: " + status.ToString());
                std::string resBuf = callback(jsonStr);
                status = google::protobuf::util::JsonStringToMessage(resBuf, &protoRes);
                if (!status.ok())
                    throw std::runtime_error("{0} parse response failed: " + status.ToString());
            }}
            res = {0}ResponseCoverToRos(protoRes);
            return true;
        }}));
    }}
    ~{0}Advertiser() override = default;
private:
    ros::NodeHandle nh{{}};
    ros::ServiceServer server{{}};
}};
'''.format(srvName, hybridMsgType, rosMsgType)

externCInterface = \
'''
extern "C" {{
hybrid::SrvAdvertiser *make_service_server(const std::string &serviceName,
                                           ros::CallbackQueue* callbackQueue,
                                           bool is_protobuf,
                                           const std::function<std::string(std::string)> &callback)
{{
    return new hybrid::{0}Advertiser(serviceName, callbackQueue, is_protobuf, callback);
}}

hybrid::SrvCaller *make_service_client(const std::string &serviceName, ros::CallbackQueue* callbackQueue, bool is_protobuf)
{{
    return new hybrid::{0}Caller(serviceName, callbackQueue, is_protobuf);
}}

}} // extern "C"
'''.format(srvName)

# output file
xxx_server_cpp = header + defineStart + include + xxxRequestCoverToRosDeclare + xxxResponseCoverToRosDeclare \
    + xxxRequestCoverToProtoDeclare + xxxResponseCoverToProtoDeclare + xxxCoverToRosDeclare + xxxCoverToProtoDeclare \
    + Build_xxx_SHARED_LIB_defineStart + xxxRequestCoverToRos + xxxResponseCoverToRos + xxxRequestCoverToProto \
    + xxxResponseCoverToProto + xxxCoverToRos + xxxCoverToProto + namespaceStart + classSrvAdvertiser + classSrvCaller \
    + namespaceEnd + externCInterface + Build_xxx_SHARED_LIB_defineEnd + defineEnd

with open('{}.server.cpp'.format(srvName), 'w') as f:
    f.write(xxx_server_cpp)

with open('result.txt', 'w') as f:
    f.write('srv/' + rosNamespace)
    f.write('\n')
    f.write('{}.server.cpp'.format(srvName))

