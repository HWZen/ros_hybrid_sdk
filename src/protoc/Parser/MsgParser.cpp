//
// Created by HWZen on 2022/12/12.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include "MsgParser.h"
#include "Preprocessor.h"
#include <regex>

std::vector<TypeTrail> MsgParser(const std::string &fileBuf, std::string_view fileName)
{

    auto preprocessRes = Preprocessor(fileBuf);
    auto defaultPkgName = std::regex_replace(std::string{fileName.data(), fileName.size()}, std::regex(R"(.*/(\w+)/msg/.*)"), "$1");
    std::vector<TypeTrail> res;
    for (auto &var : preprocessRes) {
        res.emplace_back(TypeTrailParser(var, defaultPkgName));
    }

    return res;

}

