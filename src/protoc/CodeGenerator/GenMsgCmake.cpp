//
// Created by HWZ on 2022/12/16.
//

#include "GenMsgCmake.h"
#include <fstream>
using namespace std::string_literals;

GenCodeResult GenCmake(const std::string &msgFileName, const std::vector<TypeTrail> &vars)
{
    std::string msgFileNamePy = "msgFileName = '" + msgFileName + "'\n";
    std::string GenCmake_py_part1 = R"(
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


)";

    std::string GenCmake_py_part2 = R"(

class FieldTypes(IntEnum):
    BuiltIn = 0x01
    Msg = 0x02
    Constexpr = 0x10
    Array = 0x20
    Vector = 0x40


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
# generated automatically by ros_hybrid_protoc on {0}
# Do not Edit!
# wrapping message: {1}/{2}
'''.format(time.asctime(time.localtime(time.time())), rosNamespace, msgName)

addLibrary = 'add_library({1} SHARED ${{CMAKE_CURRENT_SOURCE_DIR}}/msgs/{0}/{1}.pb.cc ${{CMAKE_CURRENT_SOURCE_DIR}}/msgs/{0}/{1}.server.cpp )\n'\
    .format(rosNamespace, msgName)
compileDefinition = 'target_compile_definitions({} PRIVATE -DBUILD_{}_SHARED_LIB)\n'.format(msgName, msgName.upper())

linkLibs = ''
for msgVar in msgVars:
    if msgVar.fieldType & FieldTypes.Msg:
        linkLibs += msgVar.msgType + ' '

linkLibs = 'target_link_libraries({} PRIVATE {} protobuf::libprotobuf ${{catkin_LIBRARIES}} HybridOption)\n'.format(msgName, linkLibs)

compileOption = \
    'target_compile_options({} PRIVATE -std=c++17 -fPIC -Wl,--version-script=${{CMAKE_CURRENT_SOURCE_DIR}}/serverDll.map)\n'\
        .format(msgName)

xxx_cmake = header + addLibrary + compileDefinition + linkLibs + compileOption

with open('{}.cmake'.format(msgName), 'w') \
        as f:
    f.write(xxx_cmake)

with open('result.txt', 'w') as f:
    f.write('msgs/' + rosNamespace)
    f.write('\n')
    f.write('{}.cmake'.format(msgName))

)";

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

    for (auto &var : vars) {
        msgVars += "    " + toPythonTypeTrail(var) + ",\n";
    }

    msgVars += "]\n";

    // write to file
    auto GenCmake_py = msgFileNamePy + GenCmake_py_part1 + msgVars + GenCmake_py_part2;
    std::ofstream ofs("GenCmake.py");
    ofs << GenCmake_py;
    ofs.close();

    // run python script
    auto systemRes = system("python3 GenCmake.py");
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
