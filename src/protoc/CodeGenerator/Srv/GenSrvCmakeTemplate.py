srvFileName = "home/pi/MyService.srv"  # will be replaced
import re
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

rosNamespace = re.search(r'.*/(\w+)/srv/.*', srvFileName).group(1)
rosMsgType = rosNamespace + '::' + srvName

header = '''
# generated automatically by ros_hybrid_protoc on {0}
# Do not Edit!
# wrapping message: {1}/{2}
'''.format(time.asctime(time.localtime(time.time())), rosNamespace, srvName)

targetName = 'srv' + rosNamespace + srvName
addLibrary = 'add_library({2} SHARED ${{CMAKE_CURRENT_SOURCE_DIR}}/srv/{0}/{1}.pb.cc ${{CMAKE_CURRENT_SOURCE_DIR}}/srv/{0}/{1}.server.cpp )\n' \
    .format(rosNamespace, srvName, targetName)
compileDefinition = 'target_compile_definitions({} PRIVATE -DBUILD_{}_SHARED_LIB)\n'.format(targetName, srvName.upper())


linkLibs = {'msg' + msgVar.msgPackage + msgVar.msgType for msgVar in srvVars if msgVar.fieldType & FieldTypes.Msg}

linkLibs = ' '.join(linkLibs)

linkLibs = 'target_link_libraries({} PRIVATE {} protobuf::libprotobuf ${{catkin_LIBRARIES}} HybridOption)\n'.format(targetName, linkLibs)

compileOption = \
    'target_compile_options({} PRIVATE -std=c++17 -fPIC -Wl,--version-script=${{CMAKE_CURRENT_SOURCE_DIR}}/serverDll.map)\n' \
        .format(targetName)

xxx_cmake = header + addLibrary + compileDefinition + linkLibs + compileOption

with open('{}.cmake'.format(srvName), 'w') \
        as f:
    f.write(xxx_cmake)

with open('result.txt', 'w') as f:
    f.write('srv/' + rosNamespace)
    f.write('\n')
    f.write('{}.cmake'.format(srvName))
