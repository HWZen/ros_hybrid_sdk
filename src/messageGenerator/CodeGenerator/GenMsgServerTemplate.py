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

    def addVarDef(self) -> str:
        if self.fieldType & 0x01:  # FieldTypes::Builtin
            typeName = builtInTypeCppTypeMap[self.builtinType]
        else:
            typeName = self.msgType

        if self.fieldType & 0x20:  # FieldTypes::Array
            res = 'std::array<{}, {}> {}'.format(typeName, self.arraySize, self.varName)
        elif self.fieldType & 0x40:  # FieldTypes::Vector
            res = 'std::vector<{}> {}'.format(typeName, self.varName)
        else:
            res = '{} {}'.format(typeName, self.varName)

        if self.fieldType & 0x10:  # FieldTypes::Constexpr
            res = 'static const {} = {}'.format(res, self.constData)
        return '    ' + res + ';\n'

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
]


header = '''
//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by messageGenerator on  {}
// Do not Edit!
//
// wrapping message: {}
'''.format(time.asctime(time.localtime(time.time())), msgName)

builtInTypeCppTypeMap = [
    "None",
    "uint8_t",
    "int8_t",
    "uint8_t",
    "int16_t",
    "uint16_t",
    "int32_t",
    "uint32_t",
    "int64_t",
    "uint64_t",
    "float",
    "double",
    "std::string",
    "ros::Time",
    "ros::Duration"
]

hybridMsgType = 'hybrid::' + msgName

systemRes = os.system('rosmsg show {} > {}.tmp'.format(msgName, msgName))
if systemRes != 0:
    print('rosmsg show {} failed!'.format(msgName))
    sys.exit(1)

with open('{}.tmp'.format(msgName), 'r') as f:
    [rosNamespace, rosMsgType] = re.search(r'\[(.*)]:', f.readline()).group(1).split('/')
    rosMsgType = rosNamespace + '::' + rosMsgType
os.remove('{}.tmp'.format(msgName))