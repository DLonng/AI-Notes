#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.frame_id = "turtle1";
    transformStamped.child_frame_id = "carrot1";

    // carrot1 相对于 tutle1 做了 y 轴的偏移
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 2.0;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    ros::Rate rate(10.0);
    while (node.ok()) {
        transformStamped.header.stamp = ros::Time::now();

        // 这两行表示让该 carrot1 参考系随着时间移动
        transformStamped.transform.translation.x = 2.0 * sin(ros::Time::now().toSec());
        transformStamped.transform.translation.y = 2.0 * cos(ros::Time::now().toSec());

        // 将 carrot1 相对于 tutle1 的坐标变换广播到 TF 系统中
        tfb.sendTransform(transformStamped);
        rate.sleep();
        printf("sending\n");
    }
};