//
// Created by HWZ on 2022/11/1.
//

#include <functional>
#include <dlfcn.h>
#include <sstl/sstdio.h>
#include <sys/prctl.h>

int main(int argc, char **argv)
{




    auto dlfd1 = dlopen("libros_hybird_sdk_server.so", RTLD_NOW | RTLD_LOCAL);
    if (dlfd1 == nullptr)
    {
        sstd::Println("dlopen fail!");
        sstd::Println("error: ", dlerror());
        return 1;
    }
    auto func1 = (int(*)(int,char**,const char *))dlsym(dlfd1, "export_main");
    if (func1 == nullptr)
    {
        sstd::Println("dlsym fail!");
        sstd::Println("error: ", dlerror());
        return 1;
    }

    auto dlfd2 = dlopen("libros_hybird_sdk_server1.so", RTLD_NOW | RTLD_LOCAL);
    if (dlfd2 == nullptr)
    {
        sstd::Println("dlopen fail!");
        sstd::Println("error: ", dlerror());
        return 1;
    }
    auto func2 = (int(*)(int,char**,const char *))dlsym(dlfd2, "export_main");
    if (func2 == nullptr)
    {
        sstd::Println("dlsym fail!");
        sstd::Println("error: ", dlerror());
        return 1;
    }



//    sstd::thread th1(func1, std::move(argc), std::move(argv), "clone_server");
//    sstd::thread th2(func1, std::move(argc), std::move(argv), "clone_server2");
//
//
//    th1.join();
//    auto ret = th1.getResult();
//    if(!ret.checkValid() || *(ret->val) != 0){
//        sstd::Println("clone server exit with error!");
//        sstd::Println("state: ", (char)th1.getStatus());
//        return 1;
//    }
//
//    th2.join();
//    ret = th2.getResult();
//    if(!ret.checkValid() || *(ret->val) != 0){
//        sstd::Println("clone server exit with error!");
//        sstd::Println("state: ", (char)th2.getStatus());
//        return 1;
//    }

    auto pid = fork();
    if (pid == 0){
        prctl(PR_SET_NAME, "clone_server", NULL, NULL, NULL);
        return func2(argc, argv, "clone_server");
    }
    else if (pid > 0){
        prctl(PR_SET_NAME, "main_server", NULL, NULL, NULL);
        return func2(argc, argv, "main_server");
    }
    else{
        sstd::Println("fork fail!");
        return 1;
    }

}

