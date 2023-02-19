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
srvVars = [
    TypeTrail(2, 0, 'MyMsg', 'ros_hybrid_sdk', 0, '', 'myMsg'),
    TypeTrail(2, 0, 'Header', 'std_msgs', 0, '', 'header'),
]


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
if 'srv_' + srvName not in os.listdir('.cache'):
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
