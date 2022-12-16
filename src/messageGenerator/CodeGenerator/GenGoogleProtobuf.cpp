//
// Created by HWZen on 2022/12/12.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include "GenGoogleProtobuf.h"
#include <vector>
#include <fstream>
using namespace std::string_literals;

GenCodeResult GenGoogleProtobuf(const std::string &msgFileName, const std::vector<TypeTrail> &vars)
{
    std::string msgFileNamePy = "msgFileName = '" + msgFileName + "'\n";
    auto GoogleProtobufGenerator_py_part1 = R"(
import re
import sys
import os
import time

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
)"s;

    auto GoogleProtobufGenerator_py_part2 = R"(

rosTypeBuiltInTypeProtoTypeMap = [
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

systemRes = os.system('rosmsg show {} > {}.tmp'.format(msgName, msgName))
if systemRes != 0:
    print('rosmsg show {} failed!'.format(msgName))
    sys.exit(1)

with open('{}.tmp'.format(msgName), 'r') as f:
    [rosNamespace, rosMsgType] = re.search(r'\[(.*)]:', f.readline()).group(1).split('/')
    rosMsgType = rosNamespace + '::' + rosMsgType
os.remove('{}.tmp'.format(msgName))

header = '''
// generated automatically by messageGenerator on  {0}
// Do not Edit!
// wrapping message: {1}/{2}

syntax = "proto3";

'''.format(time.asctime(time.localtime(time.time())), rosNamespace, msgName)

include = ''
for var in msgVars:
    if var.fieldType & 0x02:  # FieldTypes::Msg
        include += 'import "{0}/{1}.proto";\n'.format(var.msgPackage, var.msgType)
    if var.builtinType == 13:  # time
        include += 'import "google/protobuf/timestamp.proto";\n'
    if var.builtinType == 14:  # duration
        include += 'import "google/protobuf/duration.proto";\n'


arrayFieldOption = False
for var in msgVars:
    if var.fieldType & 0x20:  # FieldTypes::Array
        arrayFieldOption = True
        break


dataRangeFieldOption = True
for var in msgVars:
    if var.fieldType & 0x01:
        dataRangeFieldOption = True
        break

if arrayFieldOption or dataRangeFieldOption:
    include += 'import "HybridOption.proto";\n'

package = 'package hybrid.{0};\n'.format(rosNamespace)

seq = 0
messageContent = ''
for var in msgVars:
    if var.fieldType & 0x10: # FieldTypes::Const
        continue
    seq += 1
    if var.fieldType & 0x01:  # FieldTypes::Builtin
        typeName = rosTypeBuiltInTypeProtoTypeMap[var.builtinType]
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
    messageContent += '    {};\n'.format(res)


messageStartEnd = \
'''
message {0}{{
{1}
}}
'''.format(msgName, messageContent)

xxx_proto = header + include + package + messageStartEnd

with open('{}.proto'.format(msgName), 'w') as f:
    f.write(xxx_proto)

with open('result.txt', 'w') as f:
    f.write(rosNamespace)
    f.write('\n')
    f.write('{}.proto'.format(msgName))

)"s;

    std::string msgVars = R"(
msgVars = [
)";

    auto toPythonTypeTrail = [](const TypeTrail &var) -> std::string
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

    for (auto &var: vars) {
        msgVars += "    " + toPythonTypeTrail(var) + ",\n";
    }

    msgVars += "]\n";

    // write to file
    auto GoogleProtobufGenerator_py = msgFileNamePy + GoogleProtobufGenerator_py_part1 + msgVars + GoogleProtobufGenerator_py_part2;
    std::ofstream ofs("GoogleProtobufGenerator.py");
    ofs << GoogleProtobufGenerator_py;
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