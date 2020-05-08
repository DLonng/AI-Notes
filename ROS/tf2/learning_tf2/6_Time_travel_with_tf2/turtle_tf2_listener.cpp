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
            // 在时间 time 上使用从 source_frame 到 target_frame 的变换来填充 transform 参数
            //      void tf::TransformListener::lookupTransform (const std::string &target_frame, 
            //                                                   const std::string &source_frame,
            //                                                   const ros::Time &time,
            //                                                   StampedTransform &transform) const 
            // 这里参数意思即：寻找 turtle1 到 turtle2 的变换（或者理解为 turtle2 相对于 turtle1 的坐标变换）
            // target_frame: turtle2 
            // source_frame: turtle1
            // past: 获取 5.0s 之前的变换
            // ros::Duration(1.0): 寻找变换的等待时间为 1.0s
            // 结果分析：
            //        0 - 5.0s 之间：第二个乌龟无法获取到和第一个乌龟 5s 前的变换，所以第二个乌龟会乱跑 ~_~
            //            5.0s 之后：第二个乌龟会根据 5s 前相对与第一个乌龟的变换来移动
            ros::Time past = ros::Time::now() - ros::Duration(5.0);
            transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", past, ros::Duration(1.0));

            /*
              以下是 TransformListener::lookupTransform 的高级 API 用法：
                在 source_time 时间将 source_frame 到 fixed_frame 的变换用来填充 transform：source_frame -> fixed_frame -> get [transform]
                在 target_time 时间将 fixed_frame 到 target_frame 的变换链接到（这里的表述可能不是很恰当，后面应用多了再回来修改）上一步的 transform 变换中:  fix_frame -> target_frame -> add to [tranform]
                
                fixed_frame 不会随着时间而改变，最后得到的变换是从 source_frame -> target_frame，只是这里加上了获取变换的时间

                void tf::TransformListener::lookupTransform (const std::string &target_frame,
                                                             const ros::Time &target_time,
                                                             const std::string &source_frame,
                                                             const ros::Time &source_time, 
                                                             const std::string &fixed_frame, 
                                                             StampedTransform &transform) const 

                API：http://docs.ros.org/indigo/api/tf/html/c++/classtf_1_1Transformer.html#a14536fe915c0c702534409c15714aa2f
            */
            
            /*  
              以下是 tf2_ros::Buffer::lookupTransform 的高级 API 用法：
                http://wiki.ros.org/tf2/Tutorials/Time%20travel%20with%20tf2%20%28C%2B%2B%29
              
              For example:
                ros::Time now = ros::Time::now();
                ros::Time past = now - ros::Duration(5.0);
                transformStamped = tfBuffer.lookupTransform("turtle2", now,
                                                            "turtle1", past,
                                                            "world", ros::Duration(1.0));
                
                这个函数与上面的功能一样，只不过这里获取的变换用返回值返回，而不是函数引用参数
            */
            
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