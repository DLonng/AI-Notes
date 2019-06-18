# 5、uderstanding ROS Topics

ROS Topics 在 ROS 学习中非常重要，所以自己总结了下对 Topics 的学习过程。

## 0、启动环境
先启动 ros:
```shell
roscore
```
再启动乌龟节点和键盘控制乌龟节点：
```shell
rosrun turtlesim turtlesim_node
```
```shell
rosrun turtlesim turtle_teleop_key
```
当把窗口焦点定位到键盘控制节点的 Shell 后，按键盘的方向键就可以控制小乌龟了，如下：

<div  align="center">
<img src="https://dlonng.com/images/ros/topics/turtle.png"/>
</div>


## 1、什么是 ROS Topics ?
不知道你有没有思考过上面的小乌龟是如何动起来的？其实它们就是通过 ROS Topics 来传递键盘数据。

turtlesim_node 和 turtle_teleop_key 通过一个 ROS Topics 来相互通信。

turtle_teleop_key 不断的将键盘数据发布到一个 Topics 中，而 turtlesim_node 节点订阅到同一个的 Topics 来不断地接收键盘数据，并做出响应。

ROS 也给我们提供了查看节点和主题运行状态的工具 rqt_graph，来看看如何使用它。

## 2、使用 rqt_graph
如果没有安装，则先安装，我用的 K 版本：
```shell
sudo apt-get install ros-kinetic-rqt
sudo apt-get install ros-kinetic-rqt-common-plugins
```

然后启动它：
```shell
rosrun rqt_graph rqt_graph
```

<div  align="center">
<img src="https://dlonng.com/images/ros/topics/rqt_graph.png"/>
</div>

当你把鼠标放到 /turtle1/cmd_vel 上时，两个 ROS 节点分别为蓝色和绿色，而 topics 则为红色。

这两个 nodes 通过主题 /turtle1/cmd_vel 来进行通信。

## 3、使用 rostopic
ROS 提供了一个 rostopic 工具来得到 ROS topics 的信息。

### rostopic echo 
使用 echo 可以显示发布在主题中的数据：
```shell
# rostopic echo [topic]
rostopic echo /turtle1/cmd_vel
```
执行后，终端并没有任何响应，因为我们还没有给它数据，我们切换到使用键盘控制小乌龟的 Shell，移动方向键，即可看到 echo 终端有数据打印出来：
```
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
linear: 
  x: 2.0
  y: 0.0
  z: 0.0
angular: 
  x: 0.0
  y: 0.0
  z: 0.0
---
```

现在我们再次打开 rqt_graph 程序，点击左上角的刷新按钮，即可看到增加了一个新的 rostopic 节点订阅到 /turtle1/cmd_vel 主题：

<div  align="center">
<img src="https://dlonng.com/images/ros/topics/rqt_topics.png"/>
</div>

因为我们启动了 rostopic echo 程序要来获取小乌龟的数据，所以必须要订阅到对应的主题，从中获得数据才可以，这也是 rqt_graph 会多出一个 node 的原因。

### rostopic list
使用这个命令可以获取当前系统发布和订阅的所有主题：
```shell
rostopic list -h
```
保持以上的程序运行，执行以上命令：
```shell
Published topics:
 * /turtle1/color_sensor [turtlesim/Color] 1 publisher
 * /turtle1/cmd_vel [geometry_msgs/Twist] 1 publisher
 * /rosout [rosgraph_msgs/Log] 4 publishers
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher
 * /turtle1/pose [turtlesim/Pose] 1 publisher

Subscribed topics:
 * /turtle1/cmd_vel [geometry_msgs/Twist] 2 subscribers
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /statistics [rosgraph_msgs/TopicStatistics] 1 subscriber
```

### rostopic type
节点之间通过主题来进行数据通信，而且发布者和订阅者必须发送和接收相同数据类型的数据。

我们可以使用 rostopic type 来查看任何一个主题发布数据的类型，例如：
```shell
rostopic type /turtle1/cmd_vel
```
得到：
```shell
geometry_msgs/Twist
```
再使用 show 来得到更加详细的类型：
```shell
rosmsg show geometry_msgs/Twist
```
输出：
```shell
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

### rostopic pub
上一节学习了 topic 的 msg 类型，这次通过 pub 命令来向 topic 传递 msg，格式如下：
```shell
rostopic pub [topic] [msg_type] [args]
```
我们先执行下面的命令：
```shell
rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
可以看到小乌龟走了四分之一圆周：
<div  align="center">
<img src="https://dlonng.com/images/ros/topics/pub.png"/>
</div>


再加上 -r 参数来重复发送命令，频率为 1 Hz，让小乌龟一直走圆形：
```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
如下：

<div  align="center">
<img src="https://dlonng.com/images/ros/topics/pub_r.png"/>
</div>

暂时就写这些，下次见，Bye :)