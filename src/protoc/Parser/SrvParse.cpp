//
// Created by HWZ on 2023/2/1.
//

#include "Preprocessor.h"
#include "SrvParse.h"
#include <regex>
#include <stdexcept>
SrvTrial SrvParser(const std::string &fileBuf, std::string_view fileName)
{
    auto defaultPkgName = std::regex_replace(std::string{fileName.data(), fileName.size()}, std::regex(R"(.*/(\w+)/srv/.*)"), "$1");
    auto dividing_line = fileBuf.find("---");
    if (dividing_line == std::string::npos)
        throw std::runtime_error("SrvParser: dividing line not found");
    auto requestStr = std::string_view{fileBuf.data(), dividing_line};
    auto responseStr = std::string_view{fileBuf.data() + dividing_line + 3, fileBuf.size() - dividing_line - 3};
    auto preprocessRequest = Preprocessor(requestStr);
    auto preprocessResponse = Preprocessor(responseStr);
    SrvTrial res;
    for (auto &var : preprocessRequest)
        res.request.emplace_back(TypeTrailParser(var, defaultPkgName));
    for (auto &var : preprocessResponse)
        res.response.emplace_back(TypeTrailParser(var, defaultPkgName));
    return res;
}