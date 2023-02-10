srvFileName = "/home/pi/MyService.srv"  # will be replaced
import re
import sys
import os
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


# will be replaced
requestVars = [
    TypeTrail(2, 0, 'MyMsg', 'ros_hybrid_sdk', 0, '', 'myMsg'),

]
responseVars = [
    TypeTrail(2, 0, 'Header', 'std_msgs', 0, '', 'header'),
]

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

cacheFile = '.cache/srv_' + msgName
if not os.path.exists(cacheFile):
    systemRes = os.system('rossrv show {} > {}'.format(msgName, cacheFile))
    if systemRes != 0:
        print('rossrv show {} failed!'.format(msgName))
        sys.exit(1)

with open(cacheFile, 'r') as f:
    [rosNamespace, rosMsgType] = re.search(r'\[(.*)]:', f.readline()).group(1).split('/')
    rosMsgType = rosNamespace + '::' + rosMsgType

header = '''
// generated automatically by ros_hybrid_protoc on  {0}
// Do not Edit!
// wrapping message: {1}/{2}

syntax = "proto3";

'''.format(time.asctime(time.localtime(time.time())), rosNamespace, msgName)

include = ''
for var in srvVars:
    if var.fieldType & 0x02:  # FieldTypes::Msg
        include += 'import "msgs/{0}/{1}.proto";\n'.format(var.msgPackage, var.msgType)
    if var.builtinType == 13:  # time
        include += 'import "google/protobuf/timestamp.proto";\n'
    if var.builtinType == 14:  # duration
        include += 'import "google/protobuf/duration.proto";\n'

arrayFieldOption = False
for var in requestVars:
    if var.fieldType & 0x20:  # FieldTypes::Array
        arrayFieldOption = True
        break


dataRangeFieldOption = False
for var in requestVars:
    if var.fieldType & 0x01:
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
