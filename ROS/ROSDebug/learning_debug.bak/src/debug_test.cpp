#include <iostream>
#include <string>
#include <cstdio>

#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "debug_test");

    int x = 1;

    ROS_INFO("Hello GDB");

    ros::spin();
    return 0;
}









