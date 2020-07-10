上次总结了 [ROS 机器人技术 - TF 坐标系统基本概念](https://dlonng.com/posts/ros-tf2)，这次学习下如何发布静态坐标转换，静态 TF 官方教程在[这里](http://wiki.ros.org/tf2/Tutorials)。

## 一、我在项目中应用 TF 的方法

先说下这个静态坐标转换在我项目中的应用，我在建图的时候需要一个 `rslidar -> world` 的 TF 变换，但是系统 TF 树中只有师兄 lego_loam 的 `base_link -> world`的转换，所以为了获取 `rslidar -> world`，我用 Kinetic 提供的工具手动发布了一个静态的 TF：

```shell
# 欧拉角
rosrun tf2_ros static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
```

```shell
# 四元数
rosrun tf2_ros static_transform_publisher x y z qx qy qz qw frame_id child_frame_id
```

我用的第一种参数类型，因为我只是想要 `rslidar -> world` 的变换存在，不需要真实的旋转和平移，所以我把 6 个转换参数都设置为 0：

```shell
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar
```

这样我就能在系统中找到 `rslidar -> world` 了，但是这句话是在命令行中运行的，每次测试都要运行一遍很麻烦，所以我把上面这句加到我的 launch 中，跟测试的节点一同启动：

```xml
<launch>
  <!-- 其他节点 -->
	<node pkg="tf2_ros" type="static_transform_publisher" name="dlonng_static_transform_publisher" args="0 0 0 0 0 0 base_link rslidar" />
</launch>
```

以上就是这一节静态的 TF 帮我完成的工作，下面就带你手动实现一个静态 TF 发布程序，实际使用的时候不用自己写，直接使用已经提供的 `static_transform_publisher` 即可。

## 二、自己编写静态 TF 发布者

先在你的工作空间创建一个 `learning_tf2` 包，记得 `source devel/setup.zsh` ：

```shell
catkin_create_pkg learning_tf2 tf2 tf2_ros roscpp rospy turtlesim
```

然后进入这个包中，如果已经知道包名且环境变量也 `source` 了，可以直接使用 roscd 进入，不用一个个输入路径：

```
roscd learning_tf2
```

现在编写一个 `static_turtle_tf2_broadcaster.cpp` 放到 `learning_tf2/src` 目录下，代码如下：

```cpp
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

    // 创建 tf2 的广播对象
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // 创建 tf2 要广播的静态坐标变换
    geometry_msgs::TransformStamped static_transform_stamped;

    // 对坐标变换初始化
    static_transform_stamped.header.stamp = ros::Time::now();
    // 父节点
    static_transform_stamped.header.frame_id = "camera";
    // 子节点
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
```

代码很简单，我也加了关键注释，下面开始编译，首先在 `CMakeList.txt` 中最末尾添加构建规则：

```cmake
...
add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
target_link_libraries(static_turtle_tf2_broadcaster  ${catkin_LIBRARIES} )
```

开始编译：

```
catkin_make
```

基本不会遇到编译错误，因为不是用 roslaunch 启动，所以在运行之前需要先启动 ros：

```
roscore
```

然后直接 rosrun 启动这个节点：

```shell
 $ rosrun learning_tf2 static_turtle_tf2_broadcaster mystaticturtle 0 0 1 0 0 0
```

注意这里的参数没有父节点，因为我在程序里面指定父节点时 camera 了，并且参数的顺序跟 ros 提供的 `static_transform_publisher` 不太一样，不过没关系，我们实现的功能都是一样的，运行后我们可以查看下这个变换是否发布成功：

```
rostopic list

/tf_static
```

打印出这个变换的详细参数，即平移和旋转：

```shell
rostopic echo /tf_static

transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1459282870
        nsecs: 126883440
      frame_id: world
    child_frame_id: mystaticturtle
    transform:
      translation:
        x: 0.0
        y: 0.0
        z: 1.0
      rotation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
---
```

如果你也能正确输出类似的参数，那说明你发布的变换也是 ok 的，后面可以自己修改下平移和旋转参数再编译运行下基本就能搞定用法了。

## 三、强调一下下

上面的代码只是为了学习和理解 `static_transform_publisher` 原理用的，实际使用的时候还是要用 ROS 提供的发布工具，就是我文章开头介绍的 2 种方法。

OK，今天就到这，下次见。

