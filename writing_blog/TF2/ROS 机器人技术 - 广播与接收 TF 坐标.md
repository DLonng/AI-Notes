上次我们学习了 TF 的基本概念和如何发布静态的 TF 坐标：

- [ROS 机器人技术 - TF 坐标系统基本概念](https://dlonng.com/posts/ros-tf2)
- [ROS 机器人技术 - 静态 TF 坐标帧](https://dlonng.com/posts/static-tf)

这次来总结下如何发布一个自定义的 TF 坐标转换，并监听这个变换。

## 一、编写 TF 广播者

进入上次创建的 `learning_tf2` 包中：

```shell
roscd learning_tf2
```

在 `src` 下新建一个 `turtle_tf2_broadcaster.cpp` 文件，代码如下：

```cpp
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

    ros::spin();
    return 0;
};
```

这个程序的意思是订阅输入乌龟的 pose 话题，然后在 `poseCallback` 回调函数中发布 world 到乌龟的 TF 变换，注意这个程序可以接收不同乌龟的 pose 消息，只要运行时指定乌龟的名称 `turtle_name` 即可，代码注释很详细，其他的就不说了，然后添加编译规则：

```cmake
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
target_link_libraries(turtle_tf2_broadcaster ${catkin_LIBRARIES})
```

直接编译：

```shell
catkin_make
```

基本上不会出问题，为了方便启动我们在 launch 文件中启动广播者：

```xml
<launch>
     <!-- 乌龟节点 -->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

    <!-- 控制乌龟运动的键盘节点 -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    
    <!-- 线速度和角速度的定义，但是在这个例子中并没有用到哎... -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>

    <!-- 第一个乌龟的 tf 广播者节点，参数为乌龟 1 的名字 /tutle1 -->
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster" args="/turtle1" name="turtle1_tf2_broadcaster" />
    
    <!-- 第二个乌龟的 tf 广播者节点，还是用相同的节点，只不过改变了传递的参数为乌龟 2 的名字 /turtle2 --> 
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster" args="/turtle2" name="turtle2_tf2_broadcaster" />

  </launch>
```

然后就可以直接启动了：

```shell
roslaunch learning_tf2 start_demo.launch
```

为了确定是否成功广播了变换，使用下面的命令查看一个变换的输出：

```shell
rosrun tf tf_echo /world /turtle1
```

如果在控制台输出类似下面的消息，则说明变换发布成功：

```

```

下面我们来编写一个 TF 接收者来使用我们上面发布的变换。

## 二、编写 TF 接收者

同样在 src 目录下创建 `turtle_tf2_listener.cpp`，代码如下：

```cpp
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
    
    ros::Rate rate(10.0);
    while (node.ok()) {
        // 用来保存寻找的坐标变换
        geometry_msgs::TransformStamped transformStamped;
        try{
          	// 寻找坐标变换
      		  transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
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
      
        rate.sleep();
    }

    return 0;
};
```

这里关键的代码如下：

```cpp
// 保存寻找的变换
geometry_msgs::TransformStamped transformStamped;

// 寻找 turtle1 到 turtle2 的坐标变换
// target_frame: turtle2 
// source_frame: turtle1
// ros::Time(0): 获取变换的时间，
transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
```

同样添加编译规则：

```cmake
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
target_link_libraries(turtle_tf2_listener ${catkin_LIBRARIES})
```

然后编译：

```shell
catkin_make
```

在上面广播者的 launch 文件中加上接收者的启动：

```xml
<!-- 
  这个例子一共创建了 5 个节点：
    1. 乌龟节点，包含 2 个小乌龟
    2. 控制乌龟运动的键盘节点
    3. 第一个乌龟的 tf 广播者节点
    4. 第二个乌龟的 tf 广播者节点
    5. tf 坐标系统的监听节点，用来监听 2 个乌龟之间的坐标变换
-->
<launch>
     <!-- 乌龟节点，这个节点的内部应该是创建了 2 个乌龟...... -->
    <node pkg="turtlesim" type="turtlesim_node" name="sim"/>

    <!-- 控制乌龟运动的键盘节点 -->
    <node pkg="turtlesim" type="turtle_teleop_key" name="teleop" output="screen"/>
    
    <!-- 线速度和角速度的定义，但是在这个例子中并没有用到哎... -->
    <param name="scale_linear" value="2" type="double"/>
    <param name="scale_angular" value="2" type="double"/>

    <!-- 第一个乌龟的 tf 广播者节点，参数为乌龟 1 的名字 /tutle1 -->
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster" args="/turtle1" name="turtle1_tf2_broadcaster" />
    
    <!-- 第二个乌龟的 tf 广播者节点，还是用相同的节点，只不过改变了传递的参数为乌龟 2 的名字 /turtle2 --> 
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster" args="/turtle2" name="turtle2_tf2_broadcaster" />

    <!-- 启动 tf 坐标系同的监听节点 -->
    <node pkg="learning_tf2" type="turtle_tf2_listener" name="listener" />

  </launch>
```

然后启动：

```shell
roslaunch learning_tf2 start_demo.launch
```

运行时会出现 2 个小乌龟，把窗口焦点放到终端，按上下左右键会发现第二个乌龟跟随第一个乌龟运动：



运行图片



但是刚启动时终端会报个错误：

```shell
[ERROR] [1418082761.220546623]: "turtle2" passed to lookupTransform argument target_frame does not exist.
[ERROR] [1418082761.320422000]: "turtle2" passed to lookupTransform argument target_frame does not exist.
```

这是因为我们在 `turtle2` 还没有产生之前就寻找变换，导致没有找到它，为了解决这个问题可以在寻找变换前等待变换可用：

```cpp
// 第四个参数是阻塞等待的超时时间
listener.waitForTransform("/turtle2", "/turtle1", ros::Time::now(), ros::Duration(3.0));

transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
```

加上这句运行时就不会报错了，今天就写到这里，下次见：）

