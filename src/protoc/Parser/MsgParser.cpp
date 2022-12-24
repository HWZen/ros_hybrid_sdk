//
// Created by HWZen on 2022/12/12.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include "MsgParser.h"
#include "Preprocessor.h"
#include <regex>

std::vector<TypeTrail> MsgParser(const std::string &fileBuf)
{

    auto preprocessRes = Preprocessor(fileBuf);

    std::vector<TypeTrail> res;
    for (auto &var : preprocessRes) {
        res.emplace_back(TypeTrailParser(var));
    }

    return res;

}

