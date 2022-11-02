//
// Created by HWZ on 2022/11/1.
//
#include "DispatchServer.h"
#include <csignal>
int main(int argc, char **argv)
{
    signal(SIGCHLD, SIG_IGN);
    DispatchServer dispatchServer{};
    dispatchServer.init();
    dispatchServer.run();
}