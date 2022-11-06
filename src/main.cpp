//
// Created by HWZ on 2022/11/1.
//
#include "DispatchServer.h"
#include <csignal>
#include "Log.h"
int main(int argc, char **argv)
{
    signal(SIGCHLD, SIG_IGN);
    Log::init();

    DispatchServer dispatchServer{};
    dispatchServer.init();
    dispatchServer.run();
}