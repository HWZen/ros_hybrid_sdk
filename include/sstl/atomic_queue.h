//
// Created by HWZen on 2022/7/3.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef SSTL_ATOMIC_QUEUE_H
#define SSTL_ATOMIC_QUEUE_H

#include <queue>
#include <semaphore>
#include <mutex>
#include <optional>
#include <atomic>
#include "functional.h"

namespace sstd{
    //TODO: Consider replacing the container
    template<typename T>
    class atomic_queue{
    public:
        using value_type = T;
        using pVal = value_type*;
        using refVal = value_type&;

        constexpr atomic_queue() = default;

        constexpr atomic_queue(atomic_queue&&) noexcept;

        constexpr atomic_queue(const atomic_queue &) = delete;

        ~atomic_queue() = default;

        // warning: Block indefinitely
        constexpr value_type& front();

        // warning: Block indefinitely
        constexpr value_type& back();

        // warning: Block indefinitely
        constexpr void push(auto &&data);

        // warning: Block indefinitely
        constexpr value_type pop();

        constexpr std::optional<value_type> try_front();

        constexpr std::optional<value_type> try_back();

        constexpr bool try_push(weak_same_type<value_type> auto &&data);


        constexpr std::optional<value_type> try_pop();

        template <class _Rep, class _Period>
        constexpr std::optional<value_type> try_front_for(const std::chrono::duration<_Rep, _Period>);

        template <class _Rep, class _Period>
        constexpr std::optional<value_type> try_back_for(const std::chrono::duration<_Rep, _Period>);

        template <class _Rep, class _Period>
        constexpr bool try_push_for(weak_same_type<value_type> auto &&data, const std::chrono::duration<_Rep, _Period>);

        template <class _Rep, class _Period>
        constexpr std::optional<value_type> try_pop_for(const std::chrono::duration<_Rep, _Period>);

        constexpr size_t size() {
            return m_data.size();
        }

        constexpr size_t size() const{
            return m_data.size();
        }

    private:
        std::queue<value_type> m_data;
        std::binary_semaphore m_producer_lock{1};
        std::binary_semaphore m_consumer_lock{0};
    };

    template<typename T>
    constexpr atomic_queue<T>::atomic_queue(atomic_queue &&other) noexcept : m_data{std::move(other.m_data)} {}

    template<typename T>
    constexpr T &atomic_queue<T>::front() {
        m_consumer_lock.acquire();
        auto &res = m_data.front();
        m_consumer_lock.release();
        return res;
    }

    template<typename T>
    constexpr T &atomic_queue<T>::back() {
        m_consumer_lock.acquire();
        auto &res = m_data.back();
        m_consumer_lock.release();
        return res;
    }

    template<typename T>
    constexpr void atomic_queue<T>::push(auto &&data) {
        m_producer_lock.acquire();
        if(!m_data.empty())
            m_consumer_lock.acquire();
        m_data.emplace(data);
        m_consumer_lock.release();
        m_producer_lock.release();
    }

    template<typename T>
    constexpr T atomic_queue<T>::pop() {
        m_producer_lock.acquire();
        m_consumer_lock.acquire();
        auto res = std::move(m_data.front());
        m_data.pop();
        if(!m_data.empty())
            m_consumer_lock.release();
        m_producer_lock.release();
        return res;
    }

    template<typename T>
    constexpr std::optional<T> atomic_queue<T>::try_front() {
        if(m_consumer_lock.try_acquire()){
            auto &res = m_data.front();
            m_consumer_lock.release();
            return {res};
        }
        else
            return {};
    }

    template<typename T>
    constexpr std::optional<T> atomic_queue<T>::try_back() {
        if(m_consumer_lock.try_acquire()){
            auto &res = m_data.back();
            m_consumer_lock.release();
            return {res};
        }
        else
            return {};
    }

    template<typename T>
    constexpr bool atomic_queue<T>::try_push(weak_same_type<value_type> auto &&data) {
        if(!m_producer_lock.try_acquire())
            return false;
        if(!m_data.empty() && !m_consumer_lock.try_acquire()){
            m_producer_lock.release();
            return false;
        }

        m_data.push(data);
        m_consumer_lock.release();
        m_producer_lock.release();
        return true;
    }

    template<typename T>
    constexpr std::optional<T> atomic_queue<T>::try_pop() {
        if(!m_producer_lock.try_acquire())
            return {};
        if(!m_consumer_lock.try_acquire()){
            m_producer_lock.release();
            return {};
        }

        std::optional<T> res{std::move(m_data.front())};
        m_data.pop();
        if(!m_data.empty())
            m_consumer_lock.release();
        m_producer_lock.release();
        return res;
    }

    template<typename T>
    template<class _Rep, class _Period>
    constexpr std::optional<T> atomic_queue<T>::try_front_for(const std::chrono::duration<_Rep, _Period> times) {
        if(m_consumer_lock.try_acquire_for(times)){
            auto &res = m_data.front();
            m_consumer_lock.release();
            return {res};
        }
        else
            return {};
    }

    template<typename T>
    template<class _Rep, class _Period>
    constexpr std::optional<T> atomic_queue<T>::try_back_for(const std::chrono::duration<_Rep, _Period> times) {
        if(m_consumer_lock.try_acquire_for(times)){
            auto &res = m_data.back();
            m_consumer_lock.release();
            return {res};
        }
        else
            return {};
    }

    template<typename T>
    template<class _Rep, class _Period>
    constexpr bool
    atomic_queue<T>::try_push_for(weak_same_type<value_type> auto &&data, const std::chrono::duration<_Rep, _Period> times) {
        if(!m_producer_lock.try_acquire_for(times))
            return false;
        if(!m_data.empty() && !m_consumer_lock.try_acquire_for(times)){
            m_producer_lock.release();
            return false;
        }

        m_data.push(data);
        m_consumer_lock.release();
        m_producer_lock.release();
        return true;
    }

    template<typename T>
    template<class _Rep, class _Period>
    constexpr std::optional<T> atomic_queue<T>::try_pop_for(const std::chrono::duration<_Rep, _Period> times) {
        if(!m_producer_lock.try_acquire_for(times))
            return {};
        if(!m_consumer_lock.try_acquire_for(times)){
            m_producer_lock.release();
            return {};
        }

        std::optional<T> res{std::move(m_data.front())};
        m_data.pop();
        if(!m_data.empty())
            m_consumer_lock.release();
        m_producer_lock.release();
        return res;
    }


}

#endif //SSTL_ATOMIC_QUEUE_H
