---
title: Writing a Simple Publisher and Subscriber (C++)
date: 2019-06-13 15:00:00
---
# Writing a Simple Publisher and Subscriber (C++)
***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！

### 0、学习设计模式
掌握设计模式中的观察者模式，以后设计系统会用到很多设计模式。
### 1、Writing Publisher Node
ROS 系统的 Publisher 节点可以自动广播消息，核心代码：

1）ROS 初始化
```
ros::init(argc, argv, "talker");
```
2）定义主题
```
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```
3）设置发布频率 10 Hz
```
ros::Rate loop_rate(10);
```
4）发布消息
```
chatter_pub.publish(msg);
loop_rate.sleep();
```
### 2、Writing Subscriber Node
编写一个订阅者对象来接受发布者的消息，核心代码：
1）ROS 初始化
```
ros::init(argc, argv, "listener");
```
2）订阅发布者的主题
```
ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```
3）设置订阅者的回调函数
```
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```
4）自旋，等待发布者的消息到来
```
ros::spin();
```
5）订阅者接收到发布者的消息后，自动调用回调函数。
### 3、编译
先在 CMakeList.txt 文件中添加发布者和订阅者的编译选项以及依赖的库文件：
```
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```
进入主工作空间，make：
```
cd ~/catkin_ws
catkin_make
```
### 4、测试
make 之后，来测试下订阅者是否能够收到发布者的消息。

1）启动 ros
```
roscore
```
2）运行 Publisher

确定 Shell 环境：
```
cd ~/catkin_ws
source ./devel/setup.zsh
```
启动 talker：
```
rosrun beginner_tutorials talker
```
启动成功：

![talker](https://dlonng.com/images/ros/talker_listener/talker.png)

3）启动 Listener

开启新终端，确定 Shell 环境：
```
cd ~/catkin_ws
source ./devel/setup.zsh
```
启动 Listener：
```
rosrun beginner_tutorials listener
```
Listener 接收到 Publisher 的消息了：

![listener](https://dlonng.com/images/ros/talker_listener/listener.png)
搞定！

> {{ site.prompt }}

<div  align="center">
<img src="{{ site.url }}/images/wechart.jpg" width = "200" height = "200"/>