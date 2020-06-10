#include <iostream>
#include <string>
#include <cstdio>

#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/TransformStamped.h>

std::string static_turtle_name;

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_static_tf2_broadcaster");

    if(argc != 8) {
        ROS_ERROR("Invalid number of parameters\nusage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
        return -1;
    }

    if(strcmp(argv[1], "world") == 0) {
        ROS_ERROR("Your static turtle name cannot be 'world'");
        return -1;
    }

    static_turtle_name = argv[1];

    // 1. 创建 tf2 的广播对象
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // 2. 创建 tf2 要广播的静态坐标变换
    geometry_msgs::TransformStamped static_transform_stamped;

    // 3. 对坐标变换初始化
    static_transform_stamped.header.stamp = ros::Time::now();
    static_transform_stamped.header.frame_id = "camera";
    static_transform_stamped.child_frame_id = static_turtle_name;

    // 初始化 x y z
    static_transform_stamped.transform.translation.x = atof(argv[2]);
    static_transform_stamped.transform.translation.y = atof(argv[3]);
    static_transform_stamped.transform.translation.z = atof(argv[4]);

    // 初始化四元数
    tf2::Quaternion quat;
    quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));

    static_transform_stamped.transform.rotation.x = quat.x();
    static_transform_stamped.transform.rotation.y = quat.y();
    static_transform_stamped.transform.rotation.z = quat.z();
    static_transform_stamped.transform.rotation.w = quat.w();

    // tf2 广播对象发布静态坐标变换
    static_broadcaster.sendTransform(static_transform_stamped);

    ROS_INFO("Spinning until killed publishing %s to world", static_turtle_name.c_str());

    ros::spin();
    return 0;
}









