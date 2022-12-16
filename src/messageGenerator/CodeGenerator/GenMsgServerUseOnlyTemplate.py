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

####################
# xxx.h
####################

pragma_once = '#pragma once\n'

# include file
include = '#include <ros/node_handle.h>\n'
for var in msgVars:
    if var.fieldType & 0x02:  # FieldTypes::Msg
        include += '#include "../{}/{}.h"\n'.format(var.msgPackage, var.msgType)

namespaceStart = '''
namespace hybrid {
'''
namespaceEnd = '''
} // namespace hybrid
'''

structContent = ''
for var in msgVars:
    structContent += var.addVarDef()

structStartEnd = '''
    struct {}
    {{
{}
    }};
'''.format(msgName, structContent)

classPublisher = '''
    class {0}Publisher{{
    public:
        {0}Publisher(const std::string &topic, uint32_t queue_size, bool latch = false);
        void publish(const {0} &msg);

        ros::Publisher pub;
    private:
        ros::NodeHandle nh;
    }};
'''.format(msgName)

classSubscriber = '''
    class {0}Subscriber{{
    public:
        {0}Subscriber(const std::string &topic, uint32_t queue_size, const std::function<void(
                                  {0})> &callback );

        ros::Subscriber sub;
    private:
        ros::NodeHandle nh;
    }};
'''.format(msgName)

# hybrid::msg and ros::msg conversion
coverMacroStart = '''
#ifdef HYBRID_MSG_LIB

'''
coverMacroEnd = '''
#endif // HYBRID_MSG_LIB
'''

coverMsgContent = ''
for var in msgVars:
    if var.fieldType & 0x10:  # FieldTypes::Constexpr
        continue
    if var.fieldType & 0x01:  # FieldTypes::Builtin
        if var.fieldType & 0x20:  # FieldTypes::Array
            coverMsgContent += '    std::copy(Msg2.{0}.begin(), Msg2.{0}.end(), Msg1.{0}.begin());\n'.format(var.varName)
        else:
            coverMsgContent += '    Msg1.{0} = Msg2.{0};\n'.format(var.varName)
    elif var.fieldType & 0x02:  # FieldTypes::Msg
        if var.fieldType & 0x20:  # FieldTypes::Array
            coverMsgContent += \
'    for (int i = 0; i < {0}; ++i) {{ Msg1.{1}[i] = {2}Cover<typename decltype(Msg1.{1})::value_type, typename decltype(Msg2.{1})::value_type>(Msg2.{1}[i]); }}\n' \
                    .format(var.arraySize, var.varName, var.msgType)
        elif var.fieldType & 0x40:  # FieldTypes::Vector
            coverMsgContent += \
'''
    Msg1.{0}.reserve(Msg2.{0}.size());
    for (int i = 0; i < Msg2.{0}.size(); ++i) {{ 
        Msg1.{0}.push_back({1}Cover<typename decltype(Msg1.{0})::value_type, typename decltype(Msg2.{0})::value_type>(Msg2.{0}[i]));
    }}
'''.format(var.varName, var.msgType)
        else:
            coverMsgContent += \
'    Msg1.{0} = {1}Cover<decltype(Msg1.{0}), decltype(Msg2.{0})>(Msg2.{0});\n'.format(var.varName, var.msgType)

coverMsgStartEnd = '''
template <typename TypeMsg1, typename TypeMsg2>
inline TypeMsg1 {}Cover(const TypeMsg2 &Msg2)
{{
    TypeMsg1 Msg1;
{}
    return Msg1;
}}
'''.format(msgName, coverMsgContent)

xxx_h = header + pragma_once + include + namespaceStart + structStartEnd + classPublisher + \
        classSubscriber + namespaceEnd + coverMacroStart + coverMsgStartEnd + coverMacroEnd

#################
# xxx.cpp
#################

macroDefine = '#define HYBRID_MSG_LIB\n'

include = '#include "{}.h"\n'.format(msgName)
include += '#include <{}/{}.h>\n'.format(rosNamespace, msgName)
include += '\n'

PublisherConstructor = '''
{}Publisher::{}Publisher(const std::string &topic, uint32_t queue_size, bool latch)
{{
    pub = nh.advertise<{}>(topic, queue_size, latch);
}}
'''.format(hybridMsgType, msgName, rosMsgType)

PublisherPublish = '''
void {0}Publisher::publish(const {0} &msg)
{{
    pub.publish({1}Cover<{2},{0}>(msg));
}}

'''.format(hybridMsgType, msgName, rosMsgType)

SubscriberConstructor = '''
{0}Subscriber::{1}Subscriber(const std::string &topic, uint32_t queue_size, const std::function<void({0})> &callback)
{{
    sub = nh.subscribe(topic,queue_size,
                     boost::function<void(
                             const {2}::ConstPtr &)>(
                                 [callback](
                                         const {2}::ConstPtr &ros_msg)
                                 {{
                                     callback({1}Cover<{0},{2}>(*ros_msg));
                                 }}
                             )
                     );
}}
'''.format(hybridMsgType, msgName, rosMsgType)

xxx_cpp = header + macroDefine + include + PublisherConstructor + PublisherPublish + SubscriberConstructor

#################
# CMakeLists.txt
#################
CMakeLists_txt = \
'''
# generated automatically by messageGenerator on {0}
# Do not Edit!
# use it by add_subdirectory()
add_library(wrapper{1} STATIC {1}.cpp)
target_link_libraries(wrapper{1} ${{catkin_LIBRARIES}})
'''.format(time.asctime(time.localtime(time.time())), msgName)

#################
# output result
#################
with open('{}.h'.format(msgName), 'w') as f:
    f.write(xxx_h)

with open('{}.cpp'.format(msgName), 'w') as f:
    f.write(xxx_cpp)

with open('CMakeLists.txt', 'w') as f:
    f.write(CMakeLists_txt)

with open('result.txt', 'w') as f:
    f.write(rosNamespace)
    f.write('\n')
    f.write('{}.h'.format(msgName))
    f.write('\n')
    f.write('{}.cpp'.format(msgName))
    f.write('\n')
    f.write('CMakeLists.txt')
