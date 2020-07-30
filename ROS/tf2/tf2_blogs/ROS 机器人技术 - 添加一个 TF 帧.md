在实际的机器人中往往有很多个传感器，比如我们组目前用的小车上就有相机，雷达，IMU 等，为了能够在 TF 系统中找到传感器之间的转换，就需要把每个传感器的坐标系加到系统的 TF 树中，方法很简单，下面一起来学习下。

## 一、TF 树的注意事项

在实际使用和调试 TF 的时候一定要时刻记住：**TF 树中的一个节点可以有多个子节点，但是只能有一个父节点，并且 TF 树中不能出现回环！**

一个典型的 TF 树如下：

![](http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28C%2B%2B%29?action=AttachFile&do=get&target=tree.png)

这个 TF 树中有 3 个坐标系：

- world：世界坐标系
- turtle1：乌龟 1 的坐标系，父节点是 world
- turtle2：乌龟 2 的坐标系，父节点时 world

如果你再发布一个「xxx -> turtle1」的变换，那 turtle1 就有 2 个父节点，这样是不可行的，违反了 TF 树的构建规则，实际使用是一定要注意了，如果你想查看系统当前的 TF 树，使用下面的命令：

```shell
rosrun rqt_tf_tree rqt_tf_tree
```

下面来学习如何为一个节点添加子坐标系。

## 二、添加子坐标系

同样进入 `learning_tf2` 包中：

```shell
roscd learning_tf2
```

然后在 src 下新建 `frame_tf2_broadcaster.cpp` 文件，代码如下：

```cpp
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle node;

    tf2_ros::TransformBroadcaster tfb;
    geometry_msgs::TransformStamped transformStamped;
		// 指定 carrot1 的父节点时 turtle1 
    // 即添加一个新的 carrot1 子坐标系到 turtle1 
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
```

代码中最关键的就是要正确指定发布转换的 ID：

-  `frame_id`：父坐标系名称
- `child_frame_id`：子坐标系名称

然后添加编译规则：

```cmake
add_executable(frame_tf2_broadcaster src/frame_tf2_broadcaster.cpp)
target_link_libraries(frame_tf2_broadcaster ${catkin_LIBRARIES})
```

编译：

```shell
catkin_make	
```

在 launch 中添加启动：

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

    <!-- 启动 tf 坐标系统的监听节点 -->
    <node pkg="learning_tf2" type="turtle_tf2_listener" name="listener" />

    <!-- 启动新添加的 carrot1 坐标系广播者节点 -->
    <node pkg="learning_tf2" type="frame_tf2_broadcaster" name="broadcaster_frame" />
</launch>
```

开始启动节点：

```shell
roslaunch learning_tf2 start_demo.launch
```

不过你应该发现小乌龟的跟随运动跟上一个实验一模一样，我们自己添加的坐标系没有产生作用，这是为何呢？这是因为虽然我们发布了新的变换，但是我们并没有使用它，来修改下 `listener` 的代码：

```cpp
transformStamped = listener.lookupTransform("/turtle2", "/carrot1", ros::Time(0));
```

把 `/turtle1` 坐标系改为 `/carrot1`，因为我们要使用新添加的坐标系，所以寻找变换的坐标系参数就要填写新添加的坐标系名称，这样系统才能正确找到新添加的变换，再次编译重新运行：

```cpp
catkin_make

roslaunch learning_tf2 start_demo.launch
```

你应该会发现现在的小乌龟产生的跟随运动与之前不一定了，两者之间的 y 方向有一定的距离，这个距离就是我们发布变换时指定的坐标系的相对位置：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/add_tf_frame.gif)

OK！TF 系统常用的基础就学完了，目前因为项目有用到 TF，所以写了几篇基础的文章，之前没有看过的可以再回过头看下：

- [ROS 机器人技术 - TF 坐标系统基本概念](https://dlonng.com/posts/ros-tf2)
- [ROS 机器人技术 - 静态 TF 坐标帧](https://dlonng.com/posts/static-tf)
- [ROS 机器人技术 - 广播与接收 TF 变换](https://dlonng.com/posts/tf-broad-listener)