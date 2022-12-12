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
inline FieldTypes operator|(FieldTypes a, FieldTypes b){
    return static_cast<FieldTypes>(static_cast<int>(a) | static_cast<int>(b));
}
inline FieldTypes operator&(FieldTypes a, FieldTypes b){
    return static_cast<FieldTypes>(static_cast<int>(a) & static_cast<int>(b));
}
inline bool operator||(FieldTypes a, FieldTypes b){
    return static_cast<int>(a) || static_cast<int>(b);
}
inline bool operator&&(FieldTypes a, FieldTypes b){
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

TypeTrail TypeTrailParser(const std::pair<std::string, std::string> &strTypeName);

#endif //ROS_HYBRID_SDK_TYPE_H
