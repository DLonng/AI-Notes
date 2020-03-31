# 3、Creating a ROS Package
## 1、ROS package 的最小组成
一个最简单的 ROS Package 必须包含以下两个文件：
- package.xml：提供一些包的元信息
- CMakeLists.xml：CMake 的编译配置文件

ROS 的每个包都在一个单独的目录下。

## 2、Catkin WorkSpace 的包组成
一个 Catkin 工作空间中的 ROS 包组成如下：
```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```
值得注意的是：每个 package 都有独立的目录。

## 3、创建 Catkin 包
使用 catkin_create_pkg 脚本命令来创建包：
```shell
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```
例如创建 beginner_tutorials：
```shell
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

## 4、编译
首先需要 source 当前 Shell 环境：
```shell
source /opt/ros/kinetic/setup.bash
```
开始编译：
```shell
cd ~/catkin_ws
catkin_make
```
与包有关的构建文件在 build 目录下，编译产生的可执行文件和库输出在 devel 目录下。