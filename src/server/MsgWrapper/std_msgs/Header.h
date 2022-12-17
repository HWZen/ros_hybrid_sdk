
//
// Copyright (c) 2022 HWZen All rights reserved.
// MIT License
//
// generated automatically by ros_hybrid_protoc on  Fri Dec  9 22:51:43 2022
// Do not Edit!
//
// wrapping message: Header
#pragma once
#include <ros/node_handle.h>

namespace hybrid {

    struct Header
    {
    uint32_t seq;
    ros::Time stamp;
    std::string frame_id;

    };

    class HeaderPublisher{
    public:
        HeaderPublisher(const std::string &topic, uint32_t queue_size, bool latch = false);
        void publish(const Header &msg);

        ros::Publisher pub;
    private:
        ros::NodeHandle nh;
    };

    class HeaderSubscriber{
    public:
        HeaderSubscriber(const std::string &topic, uint32_t queue_size, const std::function<void(
                                  Header)> &callback );

        ros::Subscriber sub;
    private:
        ros::NodeHandle nh;
    };

} // namespace hybrid

#ifdef HYBRID_MSG_LIB


template <typename TypeMsg1, typename TypeMsg2>
inline TypeMsg1 HeaderCover(const TypeMsg2 &Msg2)
{
    TypeMsg1 Msg1;
    Msg1.seq = Msg2.seq;
    Msg1.stamp = Msg2.stamp;
    Msg1.frame_id = Msg2.frame_id;

    return Msg1;
}

#endif // HYBRID_MSG_LIB
