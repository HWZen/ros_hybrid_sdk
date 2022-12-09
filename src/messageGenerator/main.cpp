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
#include "CodeGenerator/MsgGenerator.h"
#include "Config.h"


void parseParam(int argc, char **argv);

void parseFile(const std::string &fileName, const std::string &fileContent);

int main(int argc, char **argv) try
{
    parseParam(argc, argv);

    for (const auto &input: g_config.inputs) {
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
        parseFile(input, buf);
    }

}
catch (std::exception &e) {
    std::cerr << "catch exception: " << e.what();
}

void parseFile(const std::string &fileName, const std::string &fileContent)
{

    enum class FileType
    {
        None,
        Msg,
        Srv,
        Action
    };
    // 1. specify the file type by suffix
    FileType fileType = FileType::None;
    if (auto res = fileName.find(".msg");res != std::string::npos && res == fileName.size() - 4) {
        fileType = FileType::Msg;
    } else if (res = fileName.find(".srv");res != std::string::npos && res == fileName.size() - 4) {
        fileType = FileType::Srv;
    } else if (res = fileName.find(".action");res != std::string::npos && res == fileName.size() - 7) {
        fileType = FileType::Action;
    } else {
        std::cerr << "unknown file type\n";
        return;
    }

    // 2. parse the file
    std::vector<TypeTrail> parseRes;
    switch (fileType) {
        case FileType::Msg:
            std::cout << "parse msg file\n";
            parseRes = MsgParser(fileContent);
            break;
        case FileType::Srv:
            std::cout << "parse srv file\n";
            break;
        case FileType::Action:
            std::cout << "parse action file\n";
            break;
        default:
            throw std::logic_error("unknown file type" " file: " __FILE__ " line: "s + std::to_string(__LINE__));
    }

    if (g_config.onlyServer) {
        auto res = GenMsgServerUseOnly(fileName, parseRes);
        std::string outPath = g_config.output + "/" + res.path;
        // move  files to outPath
        std::filesystem::create_directories(outPath);
        for (auto &file: res.files) {
            try {
                if (file == "CMakeLists.txt"){
                    auto systemRes = std::system(("cat "s + file + " >> "s + outPath + "/" + file).c_str());
                    if (systemRes != 0) {
                        throw std::runtime_error("cat file fail"" file: " __FILE__ " line: "s + std::to_string(__LINE__));
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
            ("server", "Generate code for server", cxxopts::value<bool>(g_config.server))
            ("client", "Generate code for client", cxxopts::value<bool>(g_config.client))
            ("o,output", "Output directory", cxxopts::value<std::string>(g_config.output), "<path>")
            ("only-for-server", "Generate code only used for server, not for transfer",
             cxxopts::value<bool>(g_config.onlyServer));


    auto result = options.parse(argc, argv);

    if (g_config.inputs.empty()) {
        std::cout << "No input files\n";
        std::cout << options.help() << "\n";
        exit(0);
    }

    if (g_config.output.empty()) {
        std::cout << "No output directory\n";
        std::cout << options.help() << "\n";
        exit(0);
    }

    if (!g_config.server && !g_config.client) {
        std::cout << "Please select at lease one between server and client\n";
        std::cout << options.help() << "\n";
        exit(0);
    }

    if (g_config.onlyServer && !g_config.server) {
        std::cout << "--only-for-server is only available for server\n";
        std::cout << options.help() << "\n";
        exit(0);
    }
}