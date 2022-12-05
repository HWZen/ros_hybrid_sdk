//
// Created by HWZen on 2022/6/15.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef MYSTL_FUNCTIONAL_H
#define MYSTL_FUNCTIONAL_H

#include <type_traits>
#ifdef _WIN32
#include <Windows.h>
#endif // _WIN32

// static_warning
#if defined(__GNUC__)
#define static_warning(foo, msg) foo __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define static_warning(foo, msg) __declspec(deprecated(msg)) foo
#else
#pragma message("static_warning is not implemented for this compiler, define it as static_assert")
#define static_warning static_assert
#endif // defined(__GNUC__)

#ifdef _WIN32
#define ONLY_WIN_NO_EXCEPT noexcept
#else
#define ONLY_WIN_NO_EXCEPT
#endif // _WIN32

#ifdef __linux__
#define ONLY_LINUX_NO_EXCEPT noexcept
#else
#define ONLY_LINUX_NO_EXCEPT
#endif // __linux__

#ifdef __GNUC__
#define sstdCdecl __attribute__((__cdecl__))
#define sstdStdcall __attribute__((__stdcall__))
#define sstdFastcall __attribute__((__fastcall__))
#elif defined(__clang__) || defined(_MSVC_LANG)
#define sstdCdecl __cdecl
#define sstdStdcall __stdcall
#define sstdFastcall __fastcall
#else
#define sstdCdecl
#define sstdStdcall
#define sstdFastcall
#endif // __GNUC__

#ifdef _WIN32
using NATIVE_THREAD_ROUTINE = LPTHREAD_START_ROUTINE;
using NATIVE_THREAD_RETURN_TYPE = DWORD;
#elif defined(__linux__)
using NATIVE_THREAD_ROUTINE = void* (*)(void*);
using NATIVE_THREAD_RETURN_TYPE = void*;
#endif // _WIN32


// concept: judge T1 is same T2 or T2's subclass
template<typename T1, typename T2>
concept  same_or_subclass = std::is_base_of<std::remove_cvref_t<T2>, std::remove_cvref_t<T1>>::value;

template<typename T1, typename T2>
concept weak_same_type = std::is_same_v<std::remove_cvref_t<T2>, std::remove_cvref_t<T1>>;

template<typename Ty>
concept sortable = requires(Ty t) {
    *t;
    t++;
    t--;
    --t;
    ++t;
    t < t;
    t > t;
};

template<typename Ty>
concept is_vector_like = requires(Ty t) {
    t.begin();
    t.end();
    *t.begin();

};



#endif //MYSTL_FUNCTIONAL_H
