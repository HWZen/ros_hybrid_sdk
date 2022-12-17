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

addLibrary = 'add_library({1} SHARED ${{CMAKE_CURRENT_SOURCE_DIR}}/{0}/{1}.pb.cc ${{CMAKE_CURRENT_SOURCE_DIR}}/{0}/{1}.server.cpp )\n'\
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
    f.write(rosNamespace)
    f.write('\n')
    f.write('{}.cmake'.format(msgName))
