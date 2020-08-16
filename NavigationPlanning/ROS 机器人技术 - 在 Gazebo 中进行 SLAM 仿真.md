## 一、编译

```shell
CMake Error at /opt/ros/kinetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
  Could not find a package configuration file provided by "gazebo_plugins"
  with any of the following names:

    gazebo_pluginsConfig.cmake
    gazebo_plugins-config.cmake

  Add the installation prefix of "gazebo_plugins" to CMAKE_PREFIX_PATH or set
  "gazebo_plugins_DIR" to a directory containing one of the above files.  If
  "gazebo_plugins" provides a separate development package or SDK, be sure it
  has been installed.
```

安装即可：

```shell
sudo apt-get install ros-kinetic-gazebo-plugins
```

参考博客：[ROS 问题及解决方案 —— 依赖包安装](https://blog.csdn.net/qq_16775293/article/details/81022602)

## 二、运行

### 2.1 运行激光雷达 gmapping

```shell
ERROR: cannot launch node of type [mbot_teleop/mbot_teleop.py]: can't locate node [mbot_teleop.py] in package [mbot_teleop]
```

ROS 中运行 Python 脚本需要添加可执行权限：

```shell
chmod a+x src/mbot_teleop/scripts/mbot_teleop.py
```

### 2.2 运行 Kinetic gmapping

```shell
ERROR: cannot launch node of type [depthimage_to_laserscan/depthimage_to_laserscan]: depthimage_to_laserscan
```

直接安装即可：

```shell
sudo apt install ros-kinetic-depthimage-to-laserscan
```

### 2.3 真实机器人

测试不了。

```shell
ERROR: cannot launch node of type [rplidar_ros/rplidarNode]: rplidar_ros
```

直接安装：

```shell
sudo apt install ros-kinetic-rplidar-ros
```

