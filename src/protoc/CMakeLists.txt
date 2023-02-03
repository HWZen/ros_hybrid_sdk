cmake_minimum_required(VERSION 3.0.2)
project(ros_hybrid_protoc)

add_executable(ros_hybrid_protoc
        main.cpp
        Parser/MsgParser.cpp
        CodeGenerator/Msg/GenMsgServerUseOnly.cpp
        CodeGenerator/Msg/GenMsgGoogleProtobuf.cpp
        Parser/typedef.cpp
        Parser/Preprocessor.cpp
        CodeGenerator/GenServerBuildPackage.cpp
        CodeGenerator/Msg/GenMsgServerCpp.cpp
        CodeGenerator/Msg/GenMsgCmake.cpp Parser/SrvParse.cpp Parser/SrvParse.h Parser/Parser.h CodeGenerator/Srv/GenSrvCmake.cpp CodeGenerator/Srv/GenSrvCmake.h CodeGenerator/Srv/GenSrvGoogleProtobuf.cpp CodeGenerator/Srv/GenSrvGoogleProtobuf.h CodeGenerator/Srv/GenSrvServerCpp.cpp CodeGenerator/Srv/GenSrvServerCpp.h)
target_compile_options(ros_hybrid_protoc PRIVATE -std=c++17)