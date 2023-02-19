//
// Created by HWZ on 2023/2/1.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_PARSER_SRVPARSE_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_PARSER_SRVPARSE_H
#include "typedef.h"

SrvTrial SrvParser(const std::string &fileBuf, std::string_view fileName);

#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_PARSER_SRVPARSE_H
