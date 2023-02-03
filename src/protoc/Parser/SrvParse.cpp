//
// Created by HWZ on 2023/2/1.
//

#include <stdexcept>
#include "Preprocessor.h"
#include "SrvParse.h"
SrvTrial SrvParser(const std::string &fileBuf)
{
    auto dividing_line = fileBuf.find("---");
    if (dividing_line == std::string::npos)
        throw std::runtime_error("SrvParser: dividing line not found");
    auto requestStr = std::string_view{fileBuf.data(), dividing_line};
    auto responseStr = std::string_view{fileBuf.data() + dividing_line + 3, fileBuf.size() - dividing_line - 3};
    auto preprocessRequest = Preprocessor(requestStr);
    auto preprocessResponse = Preprocessor(responseStr);
    SrvTrial res;
    for (auto &var : preprocessRequest)
        res.request.emplace_back(TypeTrailParser(var));
    for (auto &var : preprocessResponse)
        res.response.emplace_back(TypeTrailParser(var));
    return res;
}