//
// Created by HWZ on 2022/11/1.
//
#include "DispatchServer.h"
#include <csignal>
#include "Log.h"

int g_argc;
char **g_argv;

int main(int argc, char **argv)
{
    g_argc = argc;
    g_argv = argv;

    signal(SIGCHLD, SIG_IGN);
    Log::init();

    DispatchServer dispatchServer{};
    dispatchServer.init();
    dispatchServer.run();
}