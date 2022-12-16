//
// Created by HWZen on 2022/12/7.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
#include <cxxopts.hpp>
#include <iostream>
#include <fstream>
#include <filesystem>
#include "Parser/MsgParser.h"
#include "CodeGenerator/CodeGenerator.h"
#include "Config.h"
using namespace std::string_literals;

void parseParam(int argc, char **argv);

void parseFile(const std::string &fileName, const std::string &fileContent);

int main(int argc, char **argv) try
{
    parseParam(argc, argv);

    if (!g_config.packagePath.empty()) {
        GenServerMsgBuildPackage(g_config.packagePath);
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

    if (g_config.onlyServer){
        for (const auto &[fileName, fileContent] : files) {
            auto vars = MsgParser(fileContent);
            auto res = GenMsgServerUseOnly(fileName, vars);
            auto output = g_config.output + "/"s + res.path;
            std::filesystem::create_directories(output);
            for (const auto &file : res.files)
                std::filesystem::rename(file, output + "/"s + file);
        }
    }

    std::vector<GenCodeResult> protobufResults;
    if (g_config.genProtobuf){
        for (const auto &[fileName, fileContent] : files) {
            auto vars = MsgParser(fileContent);
            auto protobuf = GenGoogleProtobuf(fileName, vars);
            auto output = g_config.output + "/"s + protobuf.path;
            std::filesystem::create_directories(output);
            for (const auto &file : protobuf.files)
                std::filesystem::rename(file, output + "/"s + file);
            protobufResults.emplace_back(std::move(protobuf));
        }
    }

    if (g_config.buildProtobuf){
        for (const auto &protobuf : protobufResults){
            const auto &file = protobuf.files[0];
            std::string cmd;
            auto output = g_config.output + "/"s + protobuf.path + "/"s + file;
            cmd +=  g_config.protocPath + " --cpp_out="s + g_config.output;
            cmd += " ";
            cmd += "-I="s + g_config.output;
            cmd += " ";
            cmd += output;
            system(cmd.c_str());
        }
    }

    if (g_config.genServerCode){
        for (const auto &[fileName, fileContent] : files) {
            auto vars = MsgParser(fileContent);
            auto res = GenMsgServerCpp(fileName, vars);
            auto output = g_config.output + "/"s + res.path;
            std::filesystem::create_directories(output);
            for (const auto &file : res.files)
                std::filesystem::rename(file, output + "/"s + file);
        }
    }

    std::vector<GenCodeResult> cmakeResults;
    if (g_config.genCmake){
        for (const auto &[fileName, fileContent] : files) {
            auto vars = MsgParser(fileContent);
            auto res = GenCmake(fileName, vars);
            auto output = g_config.output + "/"s + res.path;
            std::filesystem::create_directories(output);
            for (const auto &file : res.files)
                std::filesystem::rename(file, output + "/"s + file);
            cmakeResults.emplace_back(std::move(res));
        }
    }

    if (g_config.server){
        std::unordered_set<std::string> includeLines;
        for (const auto &cmake : cmakeResults)
            includeLines.insert("include("s + cmake.path + "/"s + cmake.files[0] + ")"s);
        // read CMakeLists.txt
        std::ifstream ifs( g_config.output + "/CMakeLists.txt");
        if (!ifs.is_open()) {
            std::cerr << "open file " <<  g_config.output + "/CMakeLists.txt" << " fail\n";
            return 1;
        }
        // read line
        for(std::string line; std::getline(ifs, line);)
            includeLines.erase(line);
        ifs.close();

        // write line
        std::ofstream ofs( g_config.output + "/CMakeLists.txt", std::ios::app);
        ofs << "\n";
        for (const auto &line : includeLines)
            ofs << line << "\n";
        ofs.close();
    }

    if (g_config.buildServerMsg){
        std::string cmd;
        cmd += "cd "s + g_config.output + "/../../"s;
        cmd += " && catkin_make";
        system(cmd.c_str());
    }

}
catch (std::exception &e) {
    std::cerr << "catch exception: " << e.what();
}

static void MsgMode(const std::string &fileName, const std::string &fileContent);
static void SrvMode(const std::string &fileName, const std::string &fileContent);
static void ActionMode(const std::string &fileName, const std::string &fileContent);

void parseFile(const std::string &fileName, const std::string &fileContent)
{
    if (auto res = fileName.find(".msg");res != std::string::npos && res == fileName.size() - 4) {
        MsgMode(fileName, fileContent);
    } else if (res = fileName.find(".srv");res != std::string::npos && res == fileName.size() - 4) {
        SrvMode(fileName, fileContent);
    } else if (res = fileName.find(".action");res != std::string::npos && res == fileName.size() - 7) {
        ActionMode(fileName, fileContent);
    } else {
        std::cerr << "unknown file type\n";
        return;
    }

    // TODO: gen code for transfer



    /* TODO:
     * 1. specify the file type (*.srv, *.msg) (done)
     * 2. parse them (done)
     * 3. generate code (done)
     * 4. output to file
     * 5. compile
     */

}

void parseParam(int argc, char **argv)
{

    cxxopts::Options options(argv[0], " -  ROS_Hybrid_SDK message generator");
    options.add_options()
        ("h,help", "Print help")
        ("i,input", "Input files", cxxopts::value<std::vector<std::string>>(g_config.inputs), "<xxx.msg,xxx.srv>")
        ("server",
         "Generate code for server, (will enable --create-package, --protobuf, --gen-server-code, --protobuf-cc, --build-server-msg, --gen-cmake)",
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
        ("protoc", "Specify protoc path, default is protoc", cxxopts::value<std::string>(g_config.protocPath), "<path>");

    auto result = options.parse(argc, argv);

    if (g_config.server){
        if (!std::filesystem::exists(g_config.output))
            g_config.packagePath = g_config.output;
        g_config.buildServerMsg = true;
        g_config.buildProtobuf = true;
        g_config.genProtobuf = true;
        g_config.genServerCode = true;
        g_config.genCmake = true;
    }


    // TODO: check if protobuf, protobuf-cc and build-server-msg is legal
}

static void MsgMode(const std::string &fileName, const std::string &fileContent)
{

    auto parseRes = MsgParser(fileContent);
    if (g_config.onlyServer) {
        auto res = GenMsgServerUseOnly(fileName, parseRes);
        std::string outPath = g_config.output + "/" + res.path;
        // move  files to outPath
        std::filesystem::create_directories(outPath);
        for (auto &file : res.files) {
            try {
                if (file == "CMakeLists.txt") {
                    auto systemRes = std::system(("cat "s + file + " >> "s + outPath + "/" + file).c_str());
                    if (systemRes != 0) {
                        throw std::runtime_error(
                            "cat file fail"" file: " __FILE__ " line: "s + std::to_string(__LINE__));
                    }
                    std::filesystem::remove(file);
                    continue;
                }
                auto systemRes = std::system(("mv "s + file + " "s + outPath + "/"s).c_str());
                if (systemRes != 0)
                    throw std::runtime_error(
                        "move file: " + file + " fail"" file: " __FILE__ " line: "s + std::to_string(__LINE__));
            }
            catch (std::exception &e) {
                std::cerr << e.what();
            }
        }
    } else if (g_config.server) {
        // if outPath not exist, create it as ros package
        if (!std::filesystem::exists(g_config.output))
            GenServerMsgBuildPackage(g_config.output);
        // 1. generate xxx.proto
        auto res = GenGoogleProtobuf(fileName, parseRes);
        // TODO: in this case, add a param: ros_package_dir, same as -o
        std::string outPath = g_config.output + "/" + res.path;
        // move  files to outPath
        for (auto &file : res.files)
            std::filesystem::rename(file, outPath + "/"s + file);
        // 2. generate xxx.h, xxx.cpp

    }
}

void SrvMode(const std::string &fileName, const std::string &fileContent)
{

}

void ActionMode(const std::string &fileName, const std::string &fileContent)
{

}