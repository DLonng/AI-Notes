- GDB 命令行调试：https://www.jianshu.com/p/5e0c5aaa53c1
- ROS IDE：http://wiki.ros.org/IDEs#CLion

## 准备工作

```shell
mkdir debug_ws
cd debug_ws
mkdir src
catkin_make
```



```shell
cd debug_ws/src
catkin_create_pkg learning_debug std_msgs rospy roscpp
```





## 一、命令行 GDB 调试单独节点

在启动调试前，需要为编译规则增加调试信息的输出：

```shell
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

或者在 `CMakeLists.txt` 中增加：

```cmake
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
```

编译之后，需要先启动 roscore 才能调试，因为我们不是用 roslaunch 启动：

```shell
roscore
```

然后哦直接使用 gdb 即可调试指定节点的可执行文件：

```shell
gdb devel/lib/learning_tf2/tf2_static_broadcaster
```

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_node_gdb_shell_l.png)

使用 `l` 列出当前程序的代码区信息，使用 `b` 设置断点，使用 `info b` 查看断点：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_node_gdb_shell_b.png)

按 `r` 运行程序，`n` 下一步，`q` 退出调试：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_node_gdb_shell_r.png)

常用的 gdb 调试命令如下：

- [Linux 高级编程 - 15 个 gdb 调试基础命令](https://dlonng.com/posts/gdb)

## 二、命令行  GDB 调试 roslaunch 启动的节点

通常我们都会使用 roslaunch 启动节点，主要是为了方便传递参数，GDB 也支持在 launch 文件中配置，并随着 roslaunch 一同启动调试器，配置步骤如下：

### 2.1 增加编译调试信息

与上一节一样：

```shell
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

或者在 `CMakeLists.txt` 中添加：

```cmake
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
```

### 2.2 配置 launch 文件

在 launch 文件中要启动的节点标签后面加上如下命令：

```shell
# 调试信息输出到屏幕，并且开启新的 shell 启动 gdb 调试器，但是还没启动调试
output="screen" launch-prefix="xterm -e gdb --args
```

```shell
# 调试信息输出到屏幕，并且开启新的 shell 启动 gdb 调试器，并运行调试
output="screen" launch-prefix="xterm -e gdb -ex run --args "
```

比如我要调试一个 learning_tf2 包中的 turtle_tf2_broadcaster 节点，那我的 launch 如下：

```xml
<launch>
    <node pkg="learning_tf2" type="turtle_tf2_broadcaster" args="/turtle1" name="turtle1_tf2_broadcaster" output="screen" launch-prefix="xterm -e gdb --args"/>
</launch>
```

我用的第一种命令，也就是只启动调试器，但是不立刻开始调试，因为我还需要查看下代码，看看在哪些地方打断点等，调试程序的步骤是先打断点，再启动调试。

后面的调试方法与第一节完全相同，多调试几次就熟练了，下面我们在 VSCode 中来可视化调试一个节点。

## 三、VSCode GDB 调试单独节点

ok

## 三、VSCode GDB 调试带 ROS 参数的单独节点

ok

参数是如何传递的？

## 四、VSCode GDB 同时调试多个节点







参考博客：

- [ros 项目调试:vscode 下配置开发 ROS 项目](https://blog.csdn.net/weixin_35695879/article/details/85254422)
- [http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch Nodes in Valgrind or GDB)
- https://bluesat.com.au/a-dummys-guide-to-debugging-ros-systems/