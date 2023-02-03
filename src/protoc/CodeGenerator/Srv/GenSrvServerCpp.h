//
// Created by HWZ on 2023/2/1.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_CODEGENERATOR_SRV_GENSRVSERVERCPP_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_CODEGENERATOR_SRV_GENSRVSERVERCPP_H

#include "../GenCodeResult.h"
#include "../../Parser/typedef.h"

GenCodeResult GenSrvServerCpp(const std::string &msgFileName, const SrvTrial &vars);

#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_CODEGENERATOR_SRV_GENSRVSERVERCPP_H
