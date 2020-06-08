#include <ros/ros.h>

// 接受 tf 变换
#include <tf2_ros/transform_listener.h>

// 转换消息 
#include <geometry_msgs/TransformStamped.h>

// 发布到乌龟 2 的运动消息：角速度和线速度
#include <geometry_msgs/Twist.h>

// 再生服务
#include <turtlesim/Spawn.h>

// 实现乌龟 2 跟随乌龟 1 运动
int main(int argc, char** argv)
{
    // 当前节点的名字
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;

    // 再生服务？
    ros::service::waitForService("spawn");
    ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
    
    turtlesim::Spawn turtle;
    
    turtle.request.x = 4;
    turtle.request.y = 2;
    turtle.request.theta = 0;
    turtle.request.name = "turtle2";
    spawner.call(turtle);

    // 角速度和线速度消息发布者，用来发布计算后的新的速度消息
    ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

    // tf 变换缓存区，最多缓存 10 秒
    tf2_ros::Buffer tfBuffer;

    // 创建监听 tf 变换对象，创建完毕即开始监听，通常定义为成员变量
    tf2_ros::TransformListener tfListener(tfBuffer);
    
    // ?
    ros::Rate rate(10.0);
    
    // node.ok?
    while (node.ok()) {
        // 用来保存寻找的坐标变换
        geometry_msgs::TransformStamped transformStamped;
        
        // 寻找变换时加上 try-catch 用来捕获异常
        try {
            /*
                此行代码解决如下运行错误：
                     [ERROR] [1496887441.589043649]: “turtle2” passed to lookupTransform argument target_frame does not exist. 
                waitForTransform 函数的意思是等待直到系统中存在可用的变换
            */
            // listener.waitForTransform("/turtle2", "/turtle1", ros::Time(0), ros::Duration(3.0));
            listener.waitForTransform("/turtle2", "/carrot1",ros::Time(0), ros::Duration(3.0));

            // 寻找 turtle2 相对于 turtle1 的坐标变换
            // target_frame: turtle2 
            // source_frame: turtle1
            // ros::Time(0): 获取变换的时间，这里获取最新的变换
            // ros::Duration(0.0): 寻找变换的超时时间，默认为 0，该参数默认省略，我这里加上了
            
            // transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0), ros::Duration(0.0));
            
            // 寻找 turtle2 相对于 carrot1 的坐标变换
            transformStamped = tfBuffer.lookupTransform("/turtle2", "/carrot1", ros::Time(0), ros::Duration(0.0));
        } catch (tf2::TransformException& ex) {
            // 打印异常信息
            ROS_WARN("%s", ex.what());
            // ?
            ros::Duration(1.0).sleep();
            continue;
        }

        // 用来保存角速度和线速度
        geometry_msgs::Twist vel_msg;

        // 新的角速度为寻找到的变换角速度的 4 倍 - 使得第二个乌龟的运动轨迹转弯更快，且轨迹是弧线
        vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
        
        // 新的线速度是寻找到的变换线速度的 0.5 倍 - 使得第二个乌龟的运动速度为第一个乌龟的一半
        vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
        
        // 发布新的速度消息，乌龟 2 节点的内部订阅了这个消息，所以乌龟 2 会收到新的角速度和线速度，以此产生跟随运动
        turtle_vel.publish(vel_msg);

        // ?
        rate.sleep();
    }

    return 0;
};