//
// Created by HWZ on 2023/2/1.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_PARSER_PARSER_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_PARSER_PARSER_H
#include <stdexcept>
#include "MsgParser.h"
#include "SrvParse.h"

Trial parser(std::string_view fileName, const std::string &fileBuf){
    if(fileName.find(".srv") != std::string::npos)
        return SrvParser(fileBuf);
    else if(fileName.find(".msg") != std::string::npos)
        return MsgParser(fileBuf);
    else
        throw std::runtime_error("unknown file type: " + std::string(fileName));
}

#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_PARSER_PARSER_H
