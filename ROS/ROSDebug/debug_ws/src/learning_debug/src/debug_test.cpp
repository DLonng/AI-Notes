#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "debug_test");
    ros::NodeHandle private_node("~");

    int test_int = 0;
    private_node.param<int>("test_int", test_int, -1);
    ROS_INFO("test_int is %d", test_int);

    std::string test_string;
    private_node.param<std::string>("test_string", test_string, "");
    ROS_INFO("test_string is %s", test_string.c_str());

    int x = 1;

    ROS_INFO("Hello GDB");

    ros::spin();
    return 0;
}
