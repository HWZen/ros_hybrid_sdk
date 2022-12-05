//
// Created by HWZen on 2022/8/20.
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
// 

#ifndef SSTL_TUPLE_H
#define SSTL_TUPLE_H
#include <type_traits>

namespace sstd{
#pragma pack(1)
    template<typename ThisType, typename ...OtherTypes>
    struct tuple{
        ThisType data;
        tuple<OtherTypes...> otherTypes;

        explicit tuple(ThisType thisType, OtherTypes... otherTypes): data(ThisType(thisType)), otherTypes(otherTypes...){}

        bool operator==(const tuple<ThisType, OtherTypes...> &other) const{
            return data == other.data && otherTypes == other.otherTypes;
        }

        void destroy(){
            ~tuple();
        }

        ~tuple(){
        }
    };
    template<typename ThisType, typename ...OtherType>
    tuple(ThisType thisType, OtherType... otherTypes)->tuple<ThisType, OtherType...>;

    template<typename Ty>
    struct tuple<Ty>{
        Ty data;
        explicit tuple(Ty thisType): data(thisType) {}
        bool operator==(const tuple<Ty> &other) const{
            return data == other.data;
        }
    };

#pragma pack()

    auto& getTop(auto &&tp){
        return tp.data;
    }

    template<size_t top>
    auto& get(auto &&tp){
        if constexpr (top > 0)
            return get<top-1>(std::forward<decltype(tp)>(tp).otherTypes);
        else
            return getTop(std::forward<decltype(tp)>(tp));
    }

}

#endif //SSTL_TUPLE_H
