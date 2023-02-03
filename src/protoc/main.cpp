//
// Created by HWZen on 2022/12/7.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include <cxxopts.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "Parser/Parser.h"
#include "CodeGenerator/CodeGenerator.h"
#include "Config.h"
using namespace std::string_literals;

void parseParam(int argc, char **argv);

// TODO: cache result of system call 'rosmsg show / rossrv show'
int main(int argc, char **argv) try
{
    parseParam(argc, argv);

    if (!g_config.packagePath.empty()) {
        GenServerBuildPackage(g_config.packagePath);
    }

    std::vector<std::pair<std::string, std::string>> files;

    for (const auto &input : g_config.inputs) {
        std::cout << input << "\n";
        // open file
        std::ifstream ifs(input);
        if (!ifs.is_open()) {
            std::cerr << "open file " << input << " fail\n";
            continue;
        }
        std::string buf;
        // read all in buf
        ifs.seekg(0, std::ios::end);
        buf.reserve(ifs.tellg());
        ifs.seekg(0, std::ios::beg);
        buf.assign((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
        ifs.close();

//        parseFile(input, buf);
        files.emplace_back(input, std::move(buf));
    }

    std::vector<std::pair<std::string, Trial>> parsedFiles;
    for (const auto &[fileName, fileContent] : files){
        parsedFiles.emplace_back(fileName, parser(fileName,fileContent));
    }

    if (g_config.onlyServer) {
        if (g_config.output.empty()){
            std::cerr << "output path is empty\n";
            return 1;
        }
        for (const auto &[fileName, vars] : parsedFiles) {
            if (auto p = std::get_if<0>(&vars); p != nullptr){
                auto res = GenMsgServerUseOnly(fileName, *p);
                auto output = g_config.output + "/"s + res.path;
                std::filesystem::create_directories(output);
                for (const auto &file : res.files)
                    std::filesystem::rename(file, output + "/"s + file);
            }
        }
    }

    std::vector<GenCodeResult> protobufResults;
    if (g_config.genProtobuf) {
        if (g_config.output.empty()){
            std::cerr << "output path is empty\n";
            return 1;
        }
        for (const auto &[fileName, vars] : parsedFiles) {
            if (auto p = std::get_if<0>(&vars); p != nullptr){
                auto protobuf = GenMsgGoogleProtobuf(fileName, *p);
                auto output = g_config.output + "/"s + protobuf.path;
                std::filesystem::create_directories(output);
                for (const auto &file : protobuf.files)
                    std::filesystem::rename(file, output + "/"s + file);
                protobufResults.emplace_back(std::move(protobuf));

            } else if (auto p2 = std::get_if<1>(&vars); p2 != nullptr){
                auto protobuf = GenSrvGoogleProtobuf(fileName, *p2);
                auto output = g_config.output + "/"s + protobuf.path;
                std::filesystem::create_directories(output);
                for (const auto &file : protobuf.files)
                    std::filesystem::rename(file, output + "/"s + file);
                protobufResults.emplace_back(std::move(protobuf));
            }
        }
    }

    if (g_config.buildProtobuf) {
        if (g_config.output.empty()){
            std::cerr << "output path is empty\n";
            return 1;
        }
        for (const auto &protobuf : protobufResults) {
            const auto &file = protobuf.files[0];
            std::string cmd;
            auto output = g_config.output + "/"s + protobuf.path + "/"s + file;
            cmd += g_config.protocPath + " --cpp_out="s + g_config.output;
            cmd += " ";
            cmd += "-I="s + g_config.output;
            cmd += " ";
            cmd += output;
            system(cmd.c_str());
        }
    }

    if (g_config.genServerCode) {
        if (g_config.output.empty()){
            std::cerr << "output path is empty\n";
            return 1;
        }
        for (const auto &[fileName, vars] : parsedFiles) {
            if (auto p = std::get_if<0>(&vars); p != nullptr){
                auto res = GenMsgServerCpp(fileName, *p);
                auto output = g_config.output + "/"s + res.path;
                std::filesystem::create_directories(output);
                for (const auto &file : res.files)
                    std::filesystem::rename(file, output + "/"s + file);
            } else if (auto p2 = std::get_if<1>(&vars); p2 != nullptr){
                auto res = GenSrvServerCpp(fileName, *p2);
                auto output = g_config.output + "/"s + res.path;
                std::filesystem::create_directories(output);
                for (const auto &file : res.files)
                    std::filesystem::rename(file, output + "/"s + file);
            }
        }
    }

    std::vector<GenCodeResult> cmakeResults;
    if (g_config.genCmake) {
        if (g_config.output.empty()){
            std::cerr << "output path is empty\n";
            return 1;
        }
        for (const auto &[fileName, vars] : parsedFiles) {
            std::vector<TypeTrail> types;
            if (auto p = std::get_if<0>(&vars); p != nullptr){
                auto res = GenMsgCmake(fileName, *p);
                auto output = g_config.output + "/"s + res.path;
                std::filesystem::create_directories(output);
                for (const auto &file : res.files)
                    std::filesystem::rename(file, output + "/"s + file);
                cmakeResults.emplace_back(std::move(res));
            } else if (auto p2 = std::get_if<1>(&vars); p2 != nullptr){
                auto res = GenSrvCmake(fileName, *p2);
                auto output = g_config.output + "/"s + res.path;
                std::filesystem::create_directories(output);
                for (const auto &file : res.files)
                    std::filesystem::rename(file, output + "/"s + file);
                cmakeResults.emplace_back(std::move(res));
            }
        }
    }

    if (g_config.server) {
        std::unordered_set<std::string> includeLines;
        for (const auto &cmake : cmakeResults)
            includeLines.insert("include("s + cmake.path + "/"s + cmake.files[0] + ")"s);
        // read CMakeLists.txt
        std::ifstream ifs(g_config.output + "/CMakeLists.txt");
        if (!ifs.is_open()) {
            std::cerr << "open file " << g_config.output + "/CMakeLists.txt" << " fail\n";
            return 1;
        }
        // read line
        for (std::string line; std::getline(ifs, line);)
            includeLines.erase(line);
        ifs.close();

        // write line
        std::ofstream ofs(g_config.output + "/CMakeLists.txt", std::ios::app);
        ofs << "\n";
        for (const auto &line : includeLines)
            ofs << line << "\n";
        ofs.close();
    }

    if (g_config.buildServerMsg) {
        if (g_config.output.empty())[[unlikely]]{
            std::cerr << "output path is empty\n";
            return 1;
        }
        std::string cmd;
        cmd += "cd "s + g_config.output + "/../../"s;
        cmd += " && catkin_make";
        system(cmd.c_str());
    }

}
catch (std::exception &e) {
    std::cerr << "catch exception: " << e.what();
}


void parseParam(int argc, char **argv)
{

    cxxopts::Options options(argv[0], " -  ROS_Hybrid_SDK message generator");
    options.add_options()
        ("h,help", "Print help")
        ("i,input", "Input files", cxxopts::value<std::vector<std::string>>(g_config.inputs), "<xxx.msg,xxx.srv>")
        ("server",
         "Generate code for server, (will enable --create-package, --gen-protobuf, --gen-server-code, --protobuf-cc, --build-server-msg, --gen-cmake)",
         cxxopts::value<bool>(g_config.server))
        ("client", "Generate code for client", cxxopts::value<bool>(g_config.client))
        ("o,output", "Output directory", cxxopts::value<std::string>(g_config.output), "<path>")
        ("only-for-server",
         "Generate code only used for server, not for transfer",
         cxxopts::value<bool>(g_config.onlyServer))
        ("create-package",
         "Create ros_hybrid_dynamic_msgs",
         cxxopts::value<std::string>(g_config.packagePath),
         "<path>")
        ("gen-protobuf", "Generate protobuf code", cxxopts::value<bool>(g_config.genProtobuf))
        ("gen-server-code", "Generate server code", cxxopts::value<bool>(g_config.genServerCode))
        ("gen-cmake", "Generate msg.cmake", cxxopts::value<bool>(g_config.genCmake))
        ("build-protobuf", "Invoke protoc to build *.proto", cxxopts::value<bool>(g_config.buildProtobuf))
        ("build-server-msg", "Invoke catkin_build to gen msg.so", cxxopts::value<bool>(g_config.buildServerMsg))
        ("protoc",
         "Specify protoc path, default is protoc",
         cxxopts::value<std::string>(g_config.protocPath),
         "<path>");

    auto result = options.parse(argc, argv);

    if (result.count("help") || (g_config.inputs.empty() && g_config.packagePath.empty())) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    if (g_config.server) {
        if (g_config.output.empty())
        {
            std::cerr << "output directory is empty\n";
            exit(1);
        }
        if (!std::filesystem::exists(g_config.output))
            g_config.packagePath = g_config.output;
        g_config.buildServerMsg = true;
        g_config.buildProtobuf = true;
        g_config.genProtobuf = true;
        g_config.genServerCode = true;
        g_config.genCmake = true;
    }
}
