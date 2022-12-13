msgFileName = "/home/pi/MyMsg.msg"  # will be replaced
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


# will be replaced
msgVars = [
    TypeTrail(17, 2, '', '', 0, '1', 'DEBUG'),
    TypeTrail(17, 2, '', '', 0, '2', 'INFO'),
    TypeTrail(17, 2, '', '', 0, '4', 'WARN'),
    TypeTrail(17, 2, '', '', 0, '8', 'ERROR'),
    TypeTrail(17, 2, '', '', 0, '16', 'FATAL'),
    TypeTrail(2, 0, 'Header', 'std_msgs', 0, '', 'header'),
    TypeTrail(1, 2, '', '', 0, '', 'level'),
    TypeTrail(1, 12, '', '', 0, '', 'name'),
    TypeTrail(1, 12, '', '', 0, '', 'msg'),
    TypeTrail(1, 12, '', '', 0, '', 'file'),
    TypeTrail(1, 12, '', '', 0, '', 'function'),
    TypeTrail(1, 7, '', '', 0, '', 'line'),
    TypeTrail(65, 12, '', '', 0, '', 'topics'),
    TypeTrail(0x01 | 0x40, 6, '', '', 0, '', 'vi'),
    TypeTrail(0x01 | 0x20, 12, '', '', 5, '', 'strs'),
    TypeTrail(0x02 | 0x40, 0, 'Byte', 'std_msgs', 0, '', 'bytes'),
    TypeTrail(0x02 | 0x20, 0, 'Int32', 'std_msgs', 5, '', 'int5'),
    TypeTrail(0x01, 12, '', '', 0, '', 'stamp'),
    TypeTrail(0x01, 2, '', '', 0, '', 'data')
]

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
    f.write('{}.proto'.format(msgName))
