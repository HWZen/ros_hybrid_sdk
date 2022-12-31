cmake_minimum_required(VERSION 3.0.2)
project(ros_hybrid_protoc)

add_executable(ros_hybrid_protoc
        main.cpp
        Parser/MsgParser.cpp
        CodeGenerator/GenMsgServerUseOnly.cpp
        CodeGenerator/GenGoogleProtobuf.cpp
        Parser/typedef.cpp
        Parser/Preprocessor.cpp
        CodeGenerator/GenServerMsgBuildPackage.cpp
        CodeGenerator/GenMsgServerCpp.cpp
        CodeGenerator/GenCmake.cpp)
target_compile_options(ros_hybrid_protoc PRIVATE -std=c++17)