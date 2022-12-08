//
// Created by HWZen on 2022/12/8.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef ROS_HYBRID_SDK_PREPROCESSING_H
#define ROS_HYBRID_SDK_PREPROCESSING_H

#include <string>
#include <vector>
#include <stdexcept>

using namespace std::string_literals;

inline bool is_valid_char(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '[' ||
           c == ']' || c == '=';
}


inline std::vector<std::pair<std::string, std::string>> Preprocessing(const std::string &fileBuf)
{

    std::string buf;
    buf.reserve(fileBuf.size());
    // remove comments
    for (size_t i = 0; i < fileBuf.size(); ++i) {
        if (fileBuf[i] == '#') {
            while (fileBuf[i] != '\n') {
                ++i;
                if (i == fileBuf.size()) {
                    break;
                }
            }
        }
        buf.push_back(fileBuf[i]);
    }

    std::vector<std::pair<std::string, std::string>> res;
    auto it = buf.begin();
    for (; it < buf.end();) {

        // 1: find type word start: a char not ' ' or '\t'
        if (it < buf.end() && !is_valid_char(*it)) {
            ++it;
            continue;
        }

        // 2: find type word end
        auto it_end = it;
        while (it_end < buf.end() && is_valid_char(*it_end)) ++it_end;
        if (it_end == buf.end()) {
            throw std::runtime_error(
                    "Preprocessing: find start word end error, file: " __FILE__ " line: "s + std::to_string(__LINE__));
        }

        std::pair<std::string, std::string> var;
        var.first = {it, it_end};


        // 3: find name word start
        it = it_end;
        while (it < buf.end() && !is_valid_char(*it)) ++it;
        if (it == buf.end()) {
            throw std::runtime_error("Preprocessing: find value word start error, file: " __FILE__ " line: "s +
                                     std::to_string(__LINE__));
        }

        // 4: find name word end
        it_end = it;
        while (it_end < buf.end() && is_valid_char(*it_end)) ++it_end;

        var.second = {it, it_end};
        res.emplace_back(std::move(var));
        it = it_end;
    }
    return res;
}


#endif //ROS_HYBRID_SDK_PREPROCESSING_H
