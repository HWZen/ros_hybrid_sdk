//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_TYPE_H
#define ROS_HYBRID_SDK_TYPE_H

#include <string>
#include <unordered_map>

enum class FieldTypes : int
{
    BuiltIn          = 0x01,
    Msg              = 0x02,
    Constexpr        = 0x10,
    Array            = 0x20,
    Vector           = 0x40,

};
FieldTypes operator|(FieldTypes a, FieldTypes b){
    return static_cast<FieldTypes>(static_cast<int>(a) | static_cast<int>(b));
}
FieldTypes operator&(FieldTypes a, FieldTypes b){
    return static_cast<FieldTypes>(static_cast<int>(a) & static_cast<int>(b));
}
bool operator||(FieldTypes a, FieldTypes b){
    return static_cast<int>(a) || static_cast<int>(b);
}
bool operator&&(FieldTypes a, FieldTypes b){
    return static_cast<int>(a) && static_cast<int>(b);
}

// bing string and type
enum class BuiltInType : int
{
    None,
    Bool,
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    Float32,
    Float64,
    String,
    Time,
    Duration,
};

// bind BuiltInType and c++ type name
inline std::unordered_map<BuiltInType, std::string> BuiltInTypeCppTypeMap{
        {BuiltInType::None,     "None"},
        {BuiltInType::Bool,     "uint8_t"},
        {BuiltInType::Int8,     "int8_t"},
        {BuiltInType::UInt8,    "uint8_t"},
        {BuiltInType::Int16,    "int16_t"},
        {BuiltInType::UInt16,   "uint16_t"},
        {BuiltInType::Int32,    "int32_t"},
        {BuiltInType::UInt32,   "uint32_t"},
        {BuiltInType::Int64,    "int64_t"},
        {BuiltInType::UInt64,   "uint64_t"},
        {BuiltInType::Float32,  "float"},
        {BuiltInType::Float64,  "double"},
        {BuiltInType::String,   "std::string"},
        {BuiltInType::Time,     "ros::Time"},
        {BuiltInType::Duration, "ros::Duration"},
};

//bind ros type name and BuiltInType
inline std::unordered_map<std::string, BuiltInType> RosTypeBuiltInTypeMap{
        {"bool",     BuiltInType::Bool},
        {"int8",     BuiltInType::Int8},
        {"byte",     BuiltInType::Int8},
        {"uint8",    BuiltInType::UInt8},
        {"char",    BuiltInType::UInt8},
        {"int16",    BuiltInType::Int16},
        {"uint16",   BuiltInType::UInt16},
        {"int32",    BuiltInType::Int32},
        {"uint32",   BuiltInType::UInt32},
        {"int64",    BuiltInType::Int64},
        {"uint64",   BuiltInType::UInt64},
        {"float32",  BuiltInType::Float32},
        {"float64",  BuiltInType::Float64},
        {"string",   BuiltInType::String},
        {"time",     BuiltInType::Time},
        {"duration", BuiltInType::Duration},
};

struct TypeTrail
{
    FieldTypes fieldType{};
    BuiltInType builtInType{};
    std::string msgType{};
    std::string msgPackage{};
    size_t arraySize{};
    std::string constData{};
    std::string name{};
};



inline TypeTrail TypeTrailParser(const std::pair<std::string, std::string> &strTypeName)
{

    TypeTrail res;

    std::regex is_vector(R"(.*\[\])");
    std::regex is_array(R"(.+\[\d+\])");
    std::regex is_constexpr(R"(.*=.*)");

    auto &[strType, strName] = strTypeName;
    std::string removeSuffixType;
    // is vector or array
    if (std::regex_match(strType, is_vector))
    {
        res.fieldType = FieldTypes::Vector;
        removeSuffixType = std::regex_replace(strType, std::regex(R"((.*/)?(\w+)\[\])"), "$2");
    }
    else if (std::regex_match(strType, is_array))
    {
        res.fieldType = FieldTypes::Array;
        removeSuffixType = std::regex_replace(strType, std::regex{R"((.*/)?(\w+)\[\d+\])"}, "$2");
        res.arraySize = std::stoull(std::regex_replace(strType, std::regex{R"((.*/)?(\w+)\[(\d+)\])"}, "$3"));
    }
    else
        removeSuffixType = std::regex_replace(strType, std::regex{R"((.*/)?(\w+))"} , "$2");

    // is builtin type or msg type
    if (RosTypeBuiltInTypeMap.count(removeSuffixType))
    {
        res.fieldType = res.fieldType | FieldTypes::BuiltIn;
        res.builtInType = RosTypeBuiltInTypeMap[removeSuffixType];
    }
    else
    {
        res.msgPackage = std::regex_replace(strType, std::regex{R"((.*/)?(\w+))"} , "$1");
        if (res.msgPackage.empty()){
            // get msg package by system call 'rosmsg show'
            auto systemRes = system(("rosmsg show " + removeSuffixType + " > rosmsg.tmp").c_str());
            if (systemRes != 0)
                throw std::runtime_error("rosmsg show " + removeSuffixType + " > rosmsg.tmp failed" " file: " __FILE__ " line: "s + std::to_string(__LINE__));
            std::ifstream ifs("rosmsg.tmp");
            if (!ifs.is_open())
                throw std::runtime_error("rosmsg.tmp open failed"" file: " __FILE__ " line: "s + std::to_string(__LINE__));
            std::string line;
            std::getline(ifs, line);
            ifs.close();
            std::remove("rosmsg.tmp");
            res.msgPackage = std::regex_replace(line, std::regex{R"(\[(.*)/\w+\]:)"} , "$1");
        }

        res.fieldType = res.fieldType | FieldTypes::Msg;
        res.msgType = removeSuffixType;
    }

    // is constexpr
    if (std::regex_match(strName, is_constexpr))
    {
        res.fieldType = res.fieldType | FieldTypes::Constexpr;
        res.constData = std::regex_replace(strName, std::regex{R"(.*=[\t ]*([^ \t]*)[\t ]*)"} , "$1");
        res.name = std::regex_replace(strName, std::regex{R"((\w+)[\t ]*=.*)"} , "$1");
    }
    else
        res.name = strName;

    return res;
}


#endif //ROS_HYBRID_SDK_TYPE_H
