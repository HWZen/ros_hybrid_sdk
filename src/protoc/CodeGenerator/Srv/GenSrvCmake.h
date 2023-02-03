//
// Created by HWZ on 2023/2/1.
//

#ifndef ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_CODEGENERATOR_SRV_GENSRVCMAKE_H
#define ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_CODEGENERATOR_SRV_GENSRVCMAKE_H
#include "../GenCodeResult.h"
#include "../../Parser/typedef.h"

GenCodeResult GenSrvCmake(std::string_view srvFileName, const SrvTrial &vars);
#endif //ROS_HYBRID_SDK_SRC_ROS_HYBRID_SDK_SRC_PROTOC_CODEGENERATOR_SRV_GENSRVCMAKE_H
