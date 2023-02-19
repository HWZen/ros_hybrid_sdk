//
// Created by HWZ on 2023/2/1.
//

#include "GenSrvGoogleProtobuf.h"
#include <fstream>
#include <stdexcept>
using namespace std::string_literals;
GenCodeResult GenSrvGoogleProtobuf(const std::string &srvFileName, const SrvTrial &vars)
{
    std::string srvFileNamePy = "srvFileName = '" + srvFileName + "'\n";
    auto GoogleProtobufGenerator_py_part1 = R"(
import re
import time
from enum import IntEnum

msgName = re.search(R'(.*[/\\])?(\w+)\.srv', srvFileName).group(2)

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
)";

    auto GoogleProtobufGenerator_py_part2 = R"(

srvVars = requestVars + responseVars



def getMessageContext(typeVars):
    seq = 0
    context = ''
    for var in typeVars:
        if var.fieldType & 0x10: # FieldTypes::Const
            continue
        seq += 1
        if var.fieldType & 0x01:  # FieldTypes::Builtin
            typeName = rosTypeBuiltinTypeProtoTypeMap[var.builtinType]
        else:
            typeName = var.msgPackage + '.' + var.msgType

        if var.fieldType & 0x20 or var.fieldType & 0x40: # FieldTypes::Array or FieldTypes::Vector
            res = 'repeated {0} {1} = {2}'.format(typeName, var.varName, seq)
        else:
            res = '{0} {1} = {2}'.format(typeName, var.varName, seq)

        optionList = []
        if var.fieldType & 0x20:  # FieldTypes::Array
            optionList.append('(array_size) = {0}'.format(var.arraySize))

        if var.fieldType & 0x01:  # FieldTypes::Builtin
            if var.builtinType == 2:
                optionList.append('(data_range_min) = -128')
                optionList.append('(data_range_max) = 127')
            elif var.builtinType == 3:
                optionList.append('(data_range_min) = 0')
                optionList.append('(data_range_max) = 255')
            elif var.builtinType == 4:
                optionList.append('(data_range_min) = -32768')
                optionList.append('(data_range_max) = 32767')
            elif var.builtinType == 5:
                optionList.append('(data_range_min) = 0')
                optionList.append('(data_range_max) = 65535')

        optionStr = '['
        for option in optionList:
            optionStr += option + ', '
        optionStr = optionStr[:-2] + ']'

        if len(optionList) > 0:
            res += '  {}'.format(optionStr)
        context += '        {};\n'.format(res)

    return context


class FieldTypes(IntEnum):
    BuiltIn = 0x01
    Msg = 0x02
    Constexpr = 0x10
    Array = 0x20
    Vector = 0x40


rosTypeBuiltinTypeProtoTypeMap = [
    'none',
    'bool',
    'sint32',  # int8
    'uint32',  # uint8
    'sint32',  # int16
    'uint32',  # uint16
    'int32',
    'uint32',
    'int64',
    'uint64',
    'float',
    'double',
    'string',
    'google.protobuf.Timestamp',
    'google.protobuf.Duration',
]

rosNamespace = re.search(r'.*/(\w+)/srv/.*', srvFileName).group(1)
rosMsgType = rosNamespace + '::' + msgName

header = '''
// generated automatically by ros_hybrid_protoc on  {0}
// Do not Edit!
// wrapping message: {1}/{2}

syntax = "proto3";

'''.format(time.asctime(time.localtime(time.time())), rosNamespace, msgName)

include = set()
for var in srvVars:
    if var.fieldType & 0x02:  # FieldTypes::Msg
        include.add('import "msgs/{0}/{1}.proto";\n'.format(var.msgPackage, var.msgType))
    if var.builtinType == 13:  # time
        include.add('import "google/protobuf/timestamp.proto";\n')
    if var.builtinType == 14:  # duration
        include.add('import "google/protobuf/duration.proto";\n')
include = ''.join(include)

arrayFieldOption = False
for var in srvVars:
    if var.fieldType & 0x20 and not var.fieldType & 0x10:  # FieldTypes::Array
        arrayFieldOption = True
        break


dataRangeFieldOption = False
for var in srvVars:
    if var.fieldType & 0x01 and var.builtinType in range(2, 6) and not var.fieldType & 0x10:
        dataRangeFieldOption = True
        break

if arrayFieldOption or dataRangeFieldOption:
    include += 'import "HybridOption.proto";\n'

package = 'package hybrid.{0};\n'.format(rosNamespace)

requestMessageContent = getMessageContext(requestVars)
responseMessageContent = getMessageContext(responseVars)

message = \
'''
message {0}{{
    message Request{{
{1}
   }}
    message Response{{
{2}
    }}
    Request request = 1;
    Response response = 2;
}}
'''.format(msgName, requestMessageContent, responseMessageContent)

xxx_proto = header + include + package + message

with open('{}.proto'.format(msgName), 'w') as f:
    f.write(xxx_proto)

with open('result.txt', 'w') as f:
    f.write('srv/' + rosNamespace)
    f.write('\n')
    f.write('{}.proto'.format(msgName))

)";

    auto toPythonTypeTrial = [](const TypeTrail &var) -> std::string
    {
        std::string result = "TypeTrail(";
        result += std::to_string(static_cast<int>(var.fieldType)) + ", ";
        result += std::to_string(static_cast<int>(var.builtInType)) + ", ";
        result += "'" + var.msgType + "', ";
        result += "'" + var.msgPackage + "', ";
        result += std::to_string(var.arraySize) + ", ";
        result += "'" + var.constData + "', ";
        result += "'" + var.name + "'";
        result += ")";
        return result;
    };

    std::string requestVars = R"(
requestVars = [
)";
    for (auto &var : vars.request)
        requestVars += "    " + toPythonTypeTrial(var) + ",\n";
    requestVars += R"(
]
)";

    std::string responseVars = R"(
responseVars = [
)";
    for (auto &var : vars.response)
        responseVars += "    " + toPythonTypeTrial(var) + ",\n";
    responseVars += R"(
]
)";

    // write to file
    auto GoogleProtobuf_py = srvFileNamePy + GoogleProtobufGenerator_py_part1 + requestVars + responseVars
        + GoogleProtobufGenerator_py_part2;
    std::ofstream ofs("GoogleProtobufGenerator.py");
    ofs << GoogleProtobuf_py;
    ofs.close();

    // run python script
    auto systemRes = system("python3 GoogleProtobufGenerator.py");
    if (systemRes != 0)
        throw std::runtime_error("run python script failed!"" file: " __FILE__ " line: "s + std::to_string(__LINE__));

    // read result
    std::ifstream ifs("result.txt");
    GenCodeResult result;
    ifs >> result.path;
    for (std::string tmp; ifs >> tmp;)
        result.files.push_back(tmp);
    ifs.close();
    return result;
}