//
// Created by HWZ on 2023/2/1.
//

#include <fstream>
#include "GenSrvCmake.h"
using namespace std::string_literals;

GenCodeResult GenSrvCmake(std::string_view srvFileName, const SrvTrial &vars)
{
    std::string srvFileNamePy = "srvFileName = '" + std::string(srvFileName) + "'\n";
    std::string GenCmake_py_part1 = R"(
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

cacheFile = '.cache/srv_' + srvName
if not os.path.exists(cacheFile):
    systemRes = os.system('rossrv show {} > '.format(srvName) + cacheFile)
    if systemRes != 0:
        print('rossrv show {} failed!'.format(srvName))
        sys.exit(1)

with open(cacheFile, 'r') as f:
    [rosNamespace, rosMsgType] = re.search(r'\[(.*)]:', f.readline()).group(1).split('/')
    rosMsgType = rosNamespace + '::' + rosMsgType

header = '''
# generated automatically by ros_hybrid_protoc on {0}
# Do not Edit!
# wrapping message: {1}/{2}
'''.format(time.asctime(time.localtime(time.time())), rosNamespace, srvName)

addLibrary = 'add_library({1} SHARED ${{CMAKE_CURRENT_SOURCE_DIR}}/srv/{0}/{1}.pb.cc ${{CMAKE_CURRENT_SOURCE_DIR}}/srv/{0}/{1}.server.cpp )\n' \
    .format(rosNamespace, srvName)
compileDefinition = 'target_compile_definitions({} PRIVATE -DBUILD_{}_SHARED_LIB)\n'.format(srvName, srvName.upper())


linkLibs = {msgVar.msgType for msgVar in srvVars if msgVar.fieldType & FieldTypes.Msg}

linkLibs = ' '.join(linkLibs)

linkLibs = 'target_link_libraries({} PRIVATE {} protobuf::libprotobuf ${{catkin_LIBRARIES}} HybridOption)\n'.format(srvName, linkLibs)

compileOption = \
    'target_compile_options({} PRIVATE -std=c++17 -fPIC -Wl,--version-script=${{CMAKE_CURRENT_SOURCE_DIR}}/serverDll.map)\n' \
        .format(srvName)

xxx_cmake = header + addLibrary + compileDefinition + linkLibs + compileOption

with open('{}.cmake'.format(srvName), 'w') \
        as f:
    f.write(xxx_cmake)

with open('result.txt', 'w') as f:
    f.write('srv/' + rosNamespace)
    f.write('\n')
    f.write('{}.cmake'.format(srvName))

)";

    std::string srvVars = R"(
srvVars = [
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

    for (const auto &var : vars.request)
        srvVars += toPythonTypeTrail(var) + ",\n";
    for (const auto &var : vars.response)
        srvVars += toPythonTypeTrail(var) + ",\n";

    srvVars += "]\n";

    // write to file
    auto GenCmake_py = srvFileNamePy + GenCmake_py_part1 + srvVars + GenCmake_py_part2;
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