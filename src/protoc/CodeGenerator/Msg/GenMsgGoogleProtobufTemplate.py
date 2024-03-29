msgFileName = "/home/ros_hybrid_sdk/msg/MyMsg.msg"  # will be replaced
import re
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


rosNamespace = re.search(r'.*/(\w+)/msg/.*', msgFileName).group(1)
rosMsgType = rosNamespace + '::' + msgName

header = '''
// generated automatically by ros_hybrid_protoc on  {0}
// Do not Edit!
// wrapping message: {1}/{2}

syntax = "proto3";

'''.format(time.asctime(time.localtime(time.time())), rosNamespace, msgName)

include = set()
for var in msgVars:
    if var.fieldType & 0x02:  # FieldTypes::Msg
        include.add('import "msgs/{0}/{1}.proto";\n'.format(var.msgPackage, var.msgType))
    if var.builtinType == 13:  # time
        include.add('import "google/protobuf/timestamp.proto";\n')
    if var.builtinType == 14:  # duration
        include.add('import "google/protobuf/duration.proto";\n')


arrayFieldOption = False
for var in msgVars:
    if var.fieldType & 0x20 and not var.fieldType & 0x10:  # FieldTypes::Array
        arrayFieldOption = True
        break


dataRangeFieldOption = False
for var in msgVars:
    if var.fieldType & 0x01 and var.builtinType in range(2, 6) and not var.fieldType & 0x10:
        dataRangeFieldOption = True
        break

if arrayFieldOption or dataRangeFieldOption:
    include.add('import "HybridOption.proto";\n')

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

include = ''.join(include)

xxx_proto = header + include + package + messageStartEnd

with open('{}.proto'.format(msgName), 'w') as f:
    f.write(xxx_proto)

with open('result.txt', 'w') as f:
    f.write('msgs/' + rosNamespace)
    f.write('\n')
    f.write('{}.proto'.format(msgName))
