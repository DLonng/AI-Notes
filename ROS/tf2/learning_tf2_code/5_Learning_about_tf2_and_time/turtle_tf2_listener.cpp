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
            // 寻找 turtle2 相对于 turtle1 的坐标变换，增加了等待可用变换的阻塞等待时间
            // target_frame: turtle2 
            // source_frame: turtle1
            // ros::Time::now(): 获取变换的时间，这里从 ros::Time(0) 改为获取当前时间的变换
            //                   在实际项目中，你应该使用 ros::Time(0)，这里设置为 now 只是为了演示 tf 变换存在的时间延迟特性
            // ros::Duration(3.0): 寻找变换的等待时间
            //                     阻塞该函数直到变换可用（ROS 中等待变换可用的时间通常为几毫秒）
            //                     或者没有找到可用变换直接超时返回
            // 注意：在 tf 广播者还未发布可用变换前，该函数可能会输出 ERROR 消息，在广播者发布变换后，该函数即可正常工作，
            //      并且将阻塞等待时间设置为 3.0s 后，对乌龟节点的跟随效果影响并不大，因为这些只是毫秒级别的影响。
            // 参考链接：http://wiki.ros.org/tf2/Tutorials/tf2%20and%20time%20%28C%2B%2B%29
            transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time::now(), ros::Duration(3.0));
            
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