//
// Created by HWZen on 2022/12/12.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include "Preprocessor.h"
#include <regex>
std::vector<std::pair<std::string, std::string>> Preprocessor(std::string_view fileBuf)
{

    std::string buf;
    buf.reserve(fileBuf.size());
    // remove comments
    for (size_t i = 0; i < fileBuf.size(); ++i) {
        if (fileBuf[i] == '#') {
            while (fileBuf[i] != '\n') {
                ++i;
                if (i == fileBuf.size())
                    break;
            }
        }
        if (i == fileBuf.size())
            break;
        buf.push_back(fileBuf[i]);
    }

    std::vector<std::string> lines;
    {
        std::string line;
        for (auto c : buf) {
            if (c == '\n' || c == '\r') {
                if (line.empty())
                    continue;
                lines.emplace_back(line);
                line.clear();
            } else
                line.push_back(c);
        }
        if (!line.empty())
            lines.emplace_back(std::move(line));
    }
    std::vector<std::pair<std::string, std::string>> res;
    for (auto &line : lines) {
        if (line.empty())
            continue;
        line = std::regex_replace(line, std::regex(R"([ \t]*$)"), "");
        line = std::regex_replace(line, std::regex(R"(^[ \t]*)"), "");
        if (line.empty())
            continue;
        std::regex reg(R"(([^ \t]*)[ \t]*(.*))");
        auto type = std::regex_replace(line, reg, "$1");
        auto name = std::regex_replace(line, reg, "$2");
        res.emplace_back(type, name);
    }

    return res;
}