# Ubuntu 16.04 + ROS Kinect 安装 Robosense - 16 雷达驱动并测试

## 一、安装驱动

官方文档：[https://github.com/RoboSense-LiDAR/ros_rslidar/blob/master/doc/readme_cn.md](https://github.com/RoboSense-LiDAR/ros_rslidar/blob/master/doc/readme_cn.md)

### 1.1 下载源码

```shell
cd ~/catkin_ws/src
git clone https://github.com/RoboSense-LiDAR/ros_rslidar
```

### 1.2 安装 libpcap-dev

如果缺少这个库会提示：`pcap.h: No such file or directory`

```shell
sudo apt install libpcap-dev
```

### 1.3 更改源码权限

```shell
cd ~/catkin_ws/src/ros_rslidar/rslidar_drvier
chmod 777 cfg/*

cd ~/catkin_ws/src/ros_rslidar/rslidar_pointcloud
chmod 777 cfg/*
```

### 1.4 编译源码

```shell
cd ~/catkin_ws
catkin_make
```

## 二、配置 Ubuntu 静态 IP

默认情况：

- 雷达 IP：192.168.1.200
- Ubuntu IP：192.168.1.102

所以要将 Ubuntu 的 IP 设置为静态 192.168.1.102，子网掩码设置为 255.255.255.0，这样即可与雷达通信，设置例子如下（雷达和主机要连接到同一个路由器）：

![](https://community.bwbot.org/assets/uploads/files/1534089799300-%E7%BD%91%E7%BB%9C.png)

如果主机还需要使用网络，则可设置为本地网段：[https://blog.csdn.net/bluewhalerobot/article/details/81673063?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3](https://blog.csdn.net/bluewhalerobot/article/details/81673063?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3)

## 三、连接测试

### 3.1 硬件连接

- 给雷达供电
- 将雷达和 Ubuntu 主机连接到同一个路由器

![](http://images.ncnynl.com/ros/2018/rs-lidar-connect.png)

### 3.2 运行节点

```shell
cd ~/catkin_ws

source devel/setup.bash[zsh]

roslaunch rslidar_pointcloud rs_lidar_16.launch
```

### 3.3 Rviz 显示点云

- 设置 Fixed Frame 为 「rslidar」
- 添加 Pointcloud2 类型，并设置 Topic 为「rslidar_points」

![](https://img-blog.csdnimg.cn/20181206222724392.jpg?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2dlZXJuaXlh,size_16,color_FFFFFF,t_70)

查看发布的话题：

```shell
rostopic list

/clicked_point
/cloud_node/parameter_descriptions
/cloud_node/parameter_updates
/diagnostics
/initialpose
/move_base_simple/goal
/rosout
/rosout_agg
/rslidar_node/parameter_descriptions
/rslidar_node/parameter_updates
/rslidar_packets
/rslidar_points
/tf
/tf_static
```

## 参考博客

- https://blog.csdn.net/qq_27977711/article/details/90064963
- https://blog.csdn.net/geerniya/article/details/84866429
- https://blog.csdn.net/bluewhalerobot/article/details/81673063?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-3
- https://github.com/RoboSense-LiDAR/ros_rslidar/blob/master/doc/readme_cn.md