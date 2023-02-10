//
// Created by HWZen on 2022/12/12.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include "typedef.h"
#include <regex>
#include <fstream>
#include <filesystem>
using namespace std::string_literals;

TypeTrail TypeTrailParser(const std::pair<std::string, std::string> &strTypeName)
{

    TypeTrail res;

    std::regex is_vector(R"(.*\[\])");
    std::regex is_array(R"(.+\[\d+\])");
    std::regex is_constexpr(R"(.*=.*)");

    auto &[strType, strName] = strTypeName;
    std::string removeSuffixType;
    // is vector or array
    if (std::regex_match(strType, is_vector)) {
        res.fieldType = FieldTypes::Vector;
        removeSuffixType = std::regex_replace(strType, std::regex(R"((.*/)?(\w+)\[\])"), "$2");
    } else if (std::regex_match(strType, is_array)) {
        res.fieldType = FieldTypes::Array;
        removeSuffixType = std::regex_replace(strType, std::regex{R"((.*/)?(\w+)\[\d+\])"}, "$2");
        res.arraySize = std::stoull(std::regex_replace(strType, std::regex{R"((.*/)?(\w+)\[(\d+)\])"}, "$3"));
    } else
        removeSuffixType = std::regex_replace(strType, std::regex{R"((.*/)?(\w+))"}, "$2");

    // is builtin type or msg type
    if (RosTypeBuiltInTypeMap.count(removeSuffixType)) {
        res.fieldType = res.fieldType | FieldTypes::BuiltIn;
        res.builtInType = RosTypeBuiltInTypeMap[removeSuffixType];
    } else {
        if (std::regex_match(strType, std::regex{R"((.*)/.*)"}))
            res.msgPackage = std::regex_replace(strType, std::regex{R"((.*)/.*)"}, "$1");

        if (res.msgPackage.empty()) {
            // get msg package by system call 'rosmsg show'
            auto filePath = ".cache/msg_" + removeSuffixType;
            if(!std::filesystem::exists(filePath)) {
                auto systemRes = system(("rosmsg show " + removeSuffixType + " > " + filePath).c_str());
                if (systemRes != 0)
                    throw std::runtime_error(
                            "rosmsg show " + removeSuffixType + " > " + filePath + " failed" " file: " __FILE__ " line: "s
                            + std::to_string(__LINE__));

            }

            std::ifstream ifs(filePath);
            if (!ifs.is_open())
                throw std::runtime_error(
                    filePath + " open failed"" file: " __FILE__ " line: "s + std::to_string(__LINE__));
            std::string line;
            std::getline(ifs, line);
            ifs.close();
            res.msgPackage = std::regex_replace(line, std::regex{R"(\[(.*)/\w+\]:)"}, "$1");
        }

        res.fieldType = res.fieldType | FieldTypes::Msg;
        res.msgType = removeSuffixType;
    }

    // is constexpr
    if (std::regex_match(strName, is_constexpr)) {
        res.fieldType = res.fieldType | FieldTypes::Constexpr;
        res.constData = std::regex_replace(strName, std::regex{R"(.*=[\t ]*([^ \t]*)[\t ]*)"}, "$1");
        res.name = std::regex_replace(strName, std::regex{R"((\w+)[\t ]*=.*)"}, "$1");
    } else
        res.name = strName;

    return res;
}