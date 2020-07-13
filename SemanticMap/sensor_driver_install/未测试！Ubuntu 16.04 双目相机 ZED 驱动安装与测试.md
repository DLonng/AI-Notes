# Ubuntu 16.04 + ROS Kinect 双目相机 ZED 驱动安装与测试

按照[官网](https://www.stereolabs.com/docs/)的文档安装即可。

## 一、安装 ZED SDK

ZED SDK 允许您向应用程序添加深度，运动感应和空间 AI，它可以作为独立的安装程序提供，包括带有源代码的应用程序，工具和示例项目。

- 详细步骤在官方文档中：[https://www.stereolabs.com/docs/](https://www.stereolabs.com/docs/)

- SDK 地址：[https://www.stereolabs.com/developers/release/](https://www.stereolabs.com/developers/release/)

选择对应的 CUDA 和 Ubuntu 版本下载安装，SDK 安装过程中会自动安装 CUDA。

## 二、安装 zed-ros-wrapper

安装完 ZED SDK 后，为了在 ROS 中使用 ZED，还需要安装这个 wrapper 包，该软件包可将 ZED 立体摄像机与 ROS 一起使用，它输出摄像机的左右图像，深度图，点云，姿势信息，并支持使用多个 ZED 摄像机。

官方安装文档：[https://www.stereolabs.com/docs/ros/](https://www.stereolabs.com/docs/ros/)

编译安装：

```shell
cd ~/catkin_ws/src
git clone https://github.com/stereolabs/zed-ros-wrapper.git
cd ../

# 安装依赖包
rosdep install --from-paths src --ignore-src -r -y

catkin_make -DCMAKE_BUILD_TYPE=Release

source ./devel/setup.bash[zsh]
```

## 三、测试

注意：使用 USB 3.0 连接。

```shell
roscore
```

```shell
roslaunch zed_wrapper zed.launch
```

