//
// Created by HWZen on 2022/8/15.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//

// Thanks: https://gitee.com/xjkp2283572185/MyStd/blob/master/MyConcurrency/MyThread.hpp


#ifndef SSTL_THREAD_H
#define SSTL_THREAD_H
#include <utility>
#include <tuple>
#include "ref_ptr.h"
#include "tuple.h"
#include "functional.h"
#include <functional>
#include <any>
#include <memory>
#ifdef _WIN32

#include <Windows.h>

#elif defined(__linux__)

#include <pthread.h>
#include <unistd.h>
#include <stdexcept>


#endif // _WIN32

namespace sstd{

    inline constexpr size_t g_stackSize{ static_cast<size_t>(4e8)};

#ifdef _WIN32
    using threadFd = HANDLE;
    constexpr threadFd nullThreadFd{nullptr};
#elif defined(__linux__)
    using threadFd = pthread_t;
    const threadFd nullThreadFd{0};
#endif // _WIN32

    /*************************************
     *  C-style api
     ************************************/

    /**
     * @brief Determine threadFd is free
     * @param fd to determine
     * @return true if fd == nullThreadFd
     */
    inline bool isNullFd(threadFd fd) noexcept;

    /**
     * @brief wait for thread finish
     * @param fd to wait, if fd == nullThreadFd, nothing to do.
     */
    inline void join(threadFd fd) noexcept;

    /**
     * @brief detach thread
     * @param fd to detach, if fd == nullThreadFd, nothing to do.
     */
    inline void detach(threadFd fd) noexcept;

    /**
     * @brief Terminate the execution of the thread
     * @param fd to terminate, if fd == nullThreadFd, nothing to do.
     */
    inline void terminate(threadFd fd) ONLY_WIN_NO_EXCEPT;

    /**
     * @brief create a system thread
     * @param fn func executed by thread
     * @param args func args
     * @retval nullThreadFd if failed
     */
    template<typename _Fn, typename... _Args>
    inline threadFd createNativeFd( size_t stackSize, _Fn &&fn, _Args &&...args) noexcept;

    /**
     * @brief Get the number of current platform cpu
     * @return cpu nums
     */
    inline size_t getCpuNums() noexcept;

    namespace ThisThread
    {
        /**
         * @brief Get the fd of the current thread
         * @return fd of the current thread
         */
        inline threadFd getThreadFd() noexcept;

        /**
         * @brief Abandon the time slice of the current thread
         */
        inline void yield() ONLY_WIN_NO_EXCEPT;

        /**
         * @brief sleep the current thread
         * @param millisecond sleep for millisecond
         */
        inline void sleep(int millisecond) noexcept;

        /**
         * @brief force quit the current thread
         */
        [[noreturn]] inline void exit() ONLY_WIN_NO_EXCEPT;
    }

    /*************************************
     *  cpp OOP
     ************************************/

    /**
     * @brief thread running status
     */
    enum class threadStatus : char
    {
        UNSTART,
        RUNNING,
        FINISHED,
        DETACHED,
        EXCEPTED,   // thread throw except
        DESTROYED
    };

    /**
     * @brief thread return type
     * @tparam Ty return type
     */
    template <typename Ty>
    struct Val
    {
        using ReturnType = Ty;
        ReturnType *val = nullptr;
        ~Val(){
            delete val;
        }
    };

    template <>
    struct Val<void>
    {
        using ReturnType = void;
    };


    class ThreadToken{};
    template<typename T>
    concept Thread = std::is_base_of_v<ThreadToken, T>;



    /**
     * @brief thread support
     * @tparam stackSize thread stack size(byte)
     * @tparam Fn func executed by thread
     * @tparam Args func args
     */
    template<size_t stackSize = g_stackSize, typename Fn = void(*)(), typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    class thread : public ThreadToken
    {
    public:

        using ReturnType = std::invoke_result_t<Fn, Args...>;

        constexpr thread() noexcept = default;

        thread(const thread&) = delete;

        constexpr thread(thread&& other) noexcept {
            std::swap(this->m_threadFd, other.m_threadFd);
        }

        thread& operator=(const thread&) = delete;

        thread& operator=(thread&& other) noexcept {
            std::swap(this->m_threadFd, other.m_threadFd);
            std::swap(this->m_status, other.m_status);
            std::swap(this->m_returnVal, other.m_returnVal);
            return *this;
        }

        ~thread();

        constexpr thread(Fn fn, Args &&...args) noexcept;

        /**
         * @brief detach thread is joinablenable
         * @return true if thread is joinable
         */
        constexpr bool joinable() noexcept;

        /**
         * @brief join thread
         * @return true if thread is joinable
         */
        constexpr bool join() noexcept;

        /**
         * @brief detach thread
         * @return true if thread is joinable
         */
        constexpr bool detach() noexcept;

        /**
         * @brief terminate the thread
         * @return true if thread is running
         */
        constexpr bool terminate() ONLY_WIN_NO_EXCEPT;

        /**
         * @brief get the thread status
         * @return thread status
         */
        constexpr threadStatus getStatus() const noexcept;

        /**
         * @brief Get the exception thrown by the thread
         * @return exception
         * @retval nullptr if thread is not excepted
         */
        constexpr std::exception_ptr getException() const noexcept;

        /**
         * @brief get the thread run result
         * @return thread run result, if Fn is void, return nothing
         */
        constexpr auto getResult() const noexcept;

        /**
         * @brief create and run a thread, if instance has not created any threads
         * @return true if create new thread success
         */
        constexpr bool start(Fn &&fn, Args &&...args) noexcept;

        /**
         * @brief get the thread fd
         * @return thread fd
         */
        threadFd getRawFd() noexcept;

    protected:
        constexpr void startThread(Fn fn, Args &&...args) noexcept;

        ref_ptr<Val<ReturnType>> m_runResult{new Val<ReturnType>};

        threadStatus m_status{threadStatus::UNSTART};

        threadFd m_threadFd{nullThreadFd};

        std::exception_ptr m_exception{nullptr};

        /**
         * @brief control thread running status
         * @param fn func executed by thread
         * @param args func args
         * @return
         */
        constexpr void funcInvokeInThread(Fn fn, Args ...args) ;

        template<typename _Fn, typename... _Args>
        friend threadFd createNativeFd( size_t _stackSize, _Fn &&fn, _Args &&...args) noexcept;
        /**
         * @brief func will be passed to native create thread api, define it in there that only createNativeFd can call
         * @param pBindFn std::bind pointer, it will run like this: (*pBindFn)();
         * @return NATIVE_THREAD_RETURN_TYPE{}
         */
        template<typename Ty, size_t... Index>
        static NATIVE_THREAD_RETURN_TYPE invokeFnInThread(std::decay_t<Ty> pBindFn) ONLY_WIN_NO_EXCEPT;


        template<typename Ty, size_t... Index>
        static auto getInvokeFnInThread(Ty&&, std::index_sequence<Index...>) noexcept;
    };

    class any_thread{
        std::any Object{};
    public:
        using joinableFuncType = auto(std::any&)->bool;
        joinableFuncType* joinableFunc{ nullptr};

        using joinFuncType = auto(std::any&)->bool;
        joinFuncType* joinFunc{nullptr};

        using detachFuncType = auto(std::any&)->bool;
        detachFuncType* detachFunc{nullptr};

        using terminateFuncType = auto(std::any&)->bool;
        detachFuncType* terminateFunc{nullptr};

        using getStatusFuncType = auto(std::any&)->threadStatus;
        getStatusFuncType* getStatusFunc{nullptr};

        using getExceptionFuncType = auto(std::any&)->std::exception_ptr;
        getExceptionFuncType* getExceptionFunc{nullptr};

        any_thread() = delete;
        any_thread(Thread auto &&th);

        bool joinable() noexcept {
            return joinableFunc(Object);
        }
        bool join() noexcept {
            return joinFunc(Object);
        }
        bool detach() noexcept {
            return detachFunc(Object);
        }
        bool terminate() ONLY_WIN_NO_EXCEPT {
            return terminateFunc(Object);
        }
        threadStatus getStatus() noexcept {
            return getStatusFunc(Object);
        }
        std::exception_ptr getException() noexcept {
            return getExceptionFunc(Object);
        }
    };


    /*************************
     * implement
     ************************/

    bool isNullFd(threadFd fd) noexcept {
        return fd == nullThreadFd;
    }

    void join(threadFd fd) noexcept {
        if (isNullFd(fd))
            return;
#ifdef _WIN32
        WaitForSingleObject(fd, INFINITE);
        CloseHandle(fd);
#elif defined(__linux__)
        pthread_join(fd, nullptr);
#endif // _WIN32
    }

    void detach(threadFd fd) noexcept {
        if (isNullFd(fd))
            return;
#ifdef _WIN32
        CloseHandle(fd);
#elif defined(__linux__)
        pthread_detach(fd);
#endif // _WIN32
    }

    void terminate(threadFd fd) ONLY_WIN_NO_EXCEPT {
        if (isNullFd(fd)) [[unlikely]]
            return;
#ifdef _WIN32
        TerminateThread(fd, 0);
        CloseHandle(fd);
#elif defined(__linux__)
        // pthread_cancel is suck.
        pthread_cancel(fd);
#endif // _WIN32
    }

    // TODO: try lambda
    template<typename _Fn, typename... _Args>
    threadFd createNativeFd( size_t stackSize, _Fn &&fn, _Args &&...args) noexcept {
        auto tp = new sstd::tuple(std::forward<_Fn>(fn), std::forward<_Args>(args)...);
        auto invokeFn = sstd::thread<>::getInvokeFnInThread(tp, std::make_index_sequence<1 + sizeof...(args)>{});
#ifdef _WIN32
        threadFd fd = CreateThread(nullptr, stackSize, (LPTHREAD_START_ROUTINE)invokeFn, tp, 0, nullptr);
#elif defined(__linux__)
        pthread_attr_t attr;
        threadFd fd = 0;
        {
            auto res = pthread_attr_init(&attr);
            if (res != 0) [[unlikely]]
                return nullThreadFd;
        }
        {
            auto res = pthread_attr_setstacksize(&attr, stackSize);
            if (res != 0) [[unlikely]]
                return nullThreadFd;
        }
        {
            auto res = pthread_create(&fd, &attr, (void *(*)(void *))invokeFn, tp);
            pthread_attr_destroy(&attr);
            if (res != 0) [[unlikely]]
                return nullThreadFd;
        }
#endif // _WIN32
        return fd;
    }


    size_t getCpuNums() noexcept {
#ifdef _WIN32
        SYSTEM_INFO sysInfomation;
        GetSystemInfo(&sysInfomation);
        return sysInfomation.dwNumberOfProcessors;
#elif defined(__linux__)
        return sysconf(_SC_NPROCESSORS_ONLN);
#endif // _WIN32
    }

    threadFd ThisThread::getThreadFd() noexcept {
#ifdef _WIN32
        return GetCurrentThread();
#elif defined(__linux__)
        return pthread_self();
#endif // _WIN32
    }

    void ThisThread::yield() ONLY_WIN_NO_EXCEPT {
#ifdef _WIN32
        SwitchToThread();
#elif defined(__linux__)
        sched_yield();
#endif // _WIN32
    }

    void ThisThread::sleep(int millisecond) noexcept {
#ifdef _WIN32
        Sleep(millisecond);
#elif defined(__linux__)
        usleep(millisecond * 1000);
#endif // _WIN32
    }

    void ThisThread::exit() ONLY_WIN_NO_EXCEPT {
#ifdef _WIN32
        ExitThread(0);
#elif defined(__linux__)
        pthread_exit(nullptr);
#endif // _WIN32

    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) && (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    template<typename Ty, size_t ...Index>
    NATIVE_THREAD_RETURN_TYPE thread<stackSize, Fn, Args...>::invokeFnInThread(std::decay_t<Ty> pBindFn) ONLY_WIN_NO_EXCEPT
    {
#ifdef __linux__
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, nullptr);
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, nullptr);
#endif // __linux__
        if(pBindFn == nullptr) [[unlikely]]
            return NATIVE_THREAD_RETURN_TYPE{};
        std::invoke(std::move(get<Index>(*pBindFn))...);
        delete pBindFn;
        return NATIVE_THREAD_RETURN_TYPE{};
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    template<typename Ty, size_t ...Index>
    auto thread<stackSize, Fn, Args ...>::getInvokeFnInThread(Ty&&, std::index_sequence<Index...>) noexcept {
        return &thread::invokeFnInThread<Ty, Index...>;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) && (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr void thread<stackSize, Fn, Args...>::funcInvokeInThread(Fn fn, Args ...args) {
        m_status = threadStatus::RUNNING;
        if constexpr (!std::is_void_v<ReturnType>){
            constexpr bool isNothrow = noexcept( ReturnType{std::invoke(fn, std::forward<Args>(args)...)});
            if constexpr (isNothrow) {
                m_runResult->val = new ReturnType{std::invoke(fn, std::forward<Args>(args)...)};
                m_status = threadStatus::FINISHED;
            }
            else {
                try {
                    m_runResult->val = new ReturnType{std::invoke(fn, std::forward<Args>(args)...)};
                    m_status = threadStatus::FINISHED;
                }
                catch(std::exception&){
                    m_exception = std::current_exception();
                    m_status = threadStatus::EXCEPTED;
                }
                catch (...) {
                    if(m_status != threadStatus::DESTROYED)
                        m_status = threadStatus::EXCEPTED;
                    throw;
                }
            }
        }
        else{
            constexpr bool isNothrow = noexcept(std::invoke(fn, std::forward<Args>(args)...) );
            if constexpr (isNothrow) {
                std::invoke(fn, std::forward<Args>(args)...);
                m_status = threadStatus::FINISHED;
            }
            else {
                try {
                    std::invoke(fn, std::forward<Args>(args)...);
                    m_status = threadStatus::FINISHED;
                }
                catch(std::exception&){
                    m_exception = std::current_exception();
                    m_status = threadStatus::EXCEPTED;
                }
                catch (...) {
                    if(m_status != threadStatus::DESTROYED)
                        m_status = threadStatus::EXCEPTED;
                    throw;
                }
            }
        }
    }


    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr thread<stackSize, Fn, Args ...>::thread(Fn fn, Args&& ...args) noexcept
    {
        startThread(std::forward<Fn>(fn), std::forward<Args>(args)...);
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr void thread<stackSize, Fn, Args ...>::startThread(Fn fn, Args&& ... args) noexcept
    {
        m_threadFd = createNativeFd(stackSize, &thread::funcInvokeInThread, this, std::forward<Fn>(fn), std::forward<Args>(args)...);
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr bool thread<stackSize, Fn, Args ...>::joinable() noexcept
    {
        return !isNullFd(m_threadFd);
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr bool thread<stackSize, Fn, Args ...>::join() noexcept
    {
        if(!joinable()) [[unlikely]]
            return false;
        sstd::join(m_threadFd);
        m_threadFd = nullThreadFd;
        return true;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr bool thread<stackSize, Fn, Args ...>::detach() noexcept
    {
        if(!joinable()) [[unlikely]]
            return false;
        sstd::detach(m_threadFd);
        m_threadFd = nullThreadFd;
        m_status = threadStatus::DETACHED;
        return true;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr bool thread<stackSize, Fn, Args ...>::terminate() ONLY_WIN_NO_EXCEPT
    {
        if(!joinable() || m_status != threadStatus::RUNNING) [[unlikely]]
            return false;
        m_status = threadStatus::DESTROYED;
        sstd::terminate(m_threadFd);
        m_threadFd = nullThreadFd;
        return true;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr threadStatus thread<stackSize, Fn, Args ...>::getStatus() const noexcept
    {
        return m_status;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr auto thread<stackSize, Fn, Args ...>::getResult() const noexcept
    {
        if constexpr (std::is_void_v<ReturnType>) {
            return;
        }
        else {
            return m_status == threadStatus::FINISHED ? m_runResult : ref_ptr<Val<ReturnType>>();
        }
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr std::exception_ptr thread<stackSize, Fn, Args ...>::getException() const noexcept {
        return m_status == threadStatus::EXCEPTED ? m_exception : nullptr;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    constexpr bool thread<stackSize, Fn, Args ...>::start(Fn&& fn, Args&& ...args) noexcept
    {
        if (joinable()) [[unlikely]]
            return false;
        startThread(std::forward<Fn>(fn), std::forward<Args>(args)...);
        return true;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    threadFd thread<stackSize, Fn, Args ...>::getRawFd() noexcept
    {
        return m_threadFd;
    }

    template<size_t stackSize, typename Fn, typename... Args>
    requires(std::invocable<Fn, Args...>) &&
    (std::is_void_v<std::invoke_result_t<Fn, Args...>> || std::is_nothrow_move_constructible_v<std::invoke_result_t<Fn, Args...>>)
    thread<stackSize, Fn, Args ...>::~thread() {
        if(thread::joinable()) [[likely]]
                    thread::join();
    }

    template<typename Fn, typename... Args>
    thread(Fn, Args...) -> thread<g_stackSize, Fn, Args...>;

    any_thread::any_thread(Thread auto &&th)
    {
        using ObjectType = std::decay_t<decltype(th)>;
        using SharedType = std::shared_ptr<ObjectType>;
        this->Object = std::make_shared<ObjectType>(std::forward<decltype(th)>(th));
        this->joinableFunc = [](auto& Object){
            auto &x = *std::any_cast<SharedType&>(Object);
            return x.joinable();
        };
        this->joinFunc = [](auto& Object){
            auto &x = *std::any_cast<SharedType&>(Object);
            return x.join();
        };
        this->detachFunc = [](auto& Object){
            auto &x = *std::any_cast<SharedType&>(Object);
            return x.detach();
        };
        this->terminateFunc = [](auto& Object){
            auto &x = *std::any_cast<SharedType&>(Object);
            return x.terminate();
        };
        this->getStatusFunc = [](auto& Object){
            auto &x = *std::any_cast<SharedType>(Object);
            return x.getStatus();
        };
        this->getExceptionFunc = [](auto& Object){
            auto &x = *std::any_cast<SharedType&>(Object);
            return x.getException();
        };
    }
}

#endif //SSTL_THREAD_H
