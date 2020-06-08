#include <ros/ros.h>

// 存储要发布的坐标变换
#include <geometry_msgs/TransformStamped.h>

// 四元数
#include <tf2/LinearMath/Quaternion.h>

// 变换广播者
#include <tf2_ros/transform_broadcaster.h>

// 乌龟的位姿定义
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg) 
{
    // 创建 tf 广播对象
    static tf2_ros::TransformBroadcaster br;

    // 存储要发布的坐标变换消息
    geometry_msgs::TransformStamped transformStamped;

    // 变换的时间戳
    transformStamped.header.stamp = ros::Time::now();
    
    // 父坐标系名称
    transformStamped.header.frame_id = "world";
    
    // 当前要发布的坐标系名称 - 乌龟的名字
    transformStamped.child_frame_id = turtle_name;

    // 乌龟在二维平面运动，所以 z 坐标高度为 0
    transformStamped.transform.translation.x = msg->x;
    transformStamped.transform.translation.y = msg->y;
    transformStamped.transform.translation.z = 0.0;

    // 用四元数存储乌龟的旋转角
    tf2::Quaternion q;

    // 因为乌龟在二维平面运动，只能绕 z 轴旋转，所以 x，y 轴的旋转量为 0
    q.setRPY(0, 0, msg->theta);

    // 把四元数拷贝到要发布的坐标变换中
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    // 用 tf 广播者把订阅的乌龟位姿发布到 tf 中
    br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
    // 当前节点的名称
    ros::init(argc, argv, "my_tf2_broadcaster");

    // ?
    ros::NodeHandle private_node("~");

    // 判断当前要广播的乌龟节点名字
    if (!private_node.hasParam("turtle")) {
        // launch 文件和命令行都没有传递乌龟名称，就直接退出
        if (argc != 2) {
            ROS_ERROR("need turtle name as argument");
            return -1;
        };

        // launch 文件中如果没有定义乌龟名称，就在命令行中加上
        turtle_name = argv[1];
    } else {
        // 从 launch 文件获取乌龟名称参数
        private_node.getParam("turtle", turtle_name);
    }

    ros::NodeHandle node;

    // 订阅一个节点的 pose msg，在回调函数中广播订阅的位姿消息到 tf2 坐标系统中
    // turtle_name 为 turtle1 时广播 turtle1 的位姿到 tf 中
    // turtle_name 为 turtle2 时广播 turtle2 的位姿到 tf 中
    ros::Subscriber sub = node.subscribe(turtle_name + "/pose", 10, &poseCallback);

    // ?
    ros::spin();
    return 0;
};