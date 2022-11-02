//
// Created by HWZ on 2022/11/1.
//

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <sstl.h>

extern "C" {
    int export_main(int argc, char **argv, const char *name){
        ros::init(argc, argv, name);
        ROS_INFO("name: %s", name);

        ros::NodeHandle node1("node1");
        if (strcmp(name, "clone_server") == 0){
            auto sub = node1.subscribe("topic1", 10, boost::function<void(const std_msgs::Int64::ConstPtr &)>([](const std_msgs::Int64::ConstPtr &msg) {
                ROS_INFO("topic1: %ld", msg->data);
                if (msg->data == -1)
                    ros::shutdown();
            }));
            ros::spin();
        }
        else{
            auto pub = node1.advertise<std_msgs::Int64>("topic1", 10);
            std_msgs::Int64 msg;
            for (msg.data = 0; msg.data < 100; ++msg.data) {
                pub.publish(msg);
                ros::Duration(1).sleep();
            }
            msg.data = -1;
            pub.publish(msg);
        }
        return 0;
    }
}