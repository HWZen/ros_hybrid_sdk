//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_TYPE_H
#define ROS_HYBRID_SDK_TYPE_H

#include <string>
#include <unordered_map>

enum class FieldTypes
{
    BuiltIn,
    Constexpr,
    Array,
    Vector,
    Header
};

// bing string and type
enum class BuiltInType
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
    if (std::regex_match(strType, is_vector))
        res.fieldType = FieldTypes::Vector;
    else if (std::regex_match(strType, is_array))
        res.fieldType = FieldTypes::Array;
    else if (std::regex_match(strName, is_constexpr))
        res.fieldType = FieldTypes::Constexpr;
    else if (strType == "Header")
        return {FieldTypes::Header, BuiltInType::None, 0, "", strName};
    else
        res.fieldType = FieldTypes::BuiltIn;

    if (res.fieldType == FieldTypes::BuiltIn)
        res.builtInType = RosTypeBuiltInTypeMap[strType];
    else{
        auto strBuiltinType = std::regex_replace(strType, std::regex{R"((\w+)\[\d*\])"}, "$1");
        res.builtInType = RosTypeBuiltInTypeMap[strBuiltinType];
    }

    if (res.fieldType == FieldTypes::Array) {
        std::regex array_size(R"(.+\[(\d+)\])");
        res.arraySize = std::stoul(std::regex_replace(strType, array_size, "$1"));
    }

    if (res.fieldType == FieldTypes::Constexpr) {
        std::regex constexpr_name(R"((.*)=.*)");
        res.name = std::regex_replace(strName, constexpr_name, "$1");
    } else {
        res.name = strName;
    }

    if (res.fieldType == FieldTypes::Constexpr) {
        std::regex constexpr_data(R"(.*= *(.*) *)");
        res.constData = std::regex_replace(strName, constexpr_data, "$1");
    }

    return res;
}


#endif //ROS_HYBRID_SDK_TYPE_H
