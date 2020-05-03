# 如何在 ROS 中使用八叉树 Octomap 库？

 ## 一、Octomap 项目地址

- [Octomap](https://octomap.github.io/)
- [http://octomap.github.io/octomap/doc/](http://octomap.github.io/octomap/doc/)

## 二、Octomap 安装

### 2.1 ROS Kinect 功能包 apt 安装

```shell
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server
```

或者使用通配符全部安装：

```
sudo apt-get install ros-kinetic-octomap*
```

### 2.2 源码编译安装

获取源码：

```shell
git clone https://github.com/OctoMap/octomap
```

CMake 编译：

```shell
cd octomap
mkdir build
cd build
cmake ..
make
```

编译通过即可安装：

```shell
sudo make install
```

卸载方法，在 `build` 目录下执行：

```shell
sudo make uninstall
```

补充，如果没有可视化程序 octovis，可独立安装：

```shell
sudo apt-get install octovis
```

### 2.3 配置 CMakeList.txt

```cmake
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(xxx xxx.cpp)
target_link_libraries(${OCTOMAP_LIBRARIES})
```

### 2.4 配置 package.xml

```xml
<build_depend>octomap</build_depend>
<run_depend>octomap</run_depend>
```

## 二、Rviz 可视化 Octomap





## 三、Octomap 框架

官方文档：[Octomap](http://octomap.github.io/octomap/doc/)



![](http://octomap.github.io/octomap/doc/uml_overview.png)

主要数据结构和方法：

- octomap::OcTree：octomap 主要地图数据结构，存储 3D 占用栅格地图

  ![](http://octomap.github.io/octomap/doc/classoctomap_1_1OcTree__inherit__graph.png)

- OcTree::insertRay(...)：插入一束射线

- OcTree::insertPointCloud(...)：插入点云（在全局参考框架中）

- OcTree::search(...)，OcTree::castRay(...)：查询功能

- updateInnerOccupancy：更新地图

- updateNode(...)：插入三维空间点到地图

- leaf_iterator，tree_iterator, leaf_bbx_iterator：访问节点的迭代器

- setProHit/setProMiss：这两个函数决定了 inverse sensor model 的 log-odd 概率更新的具体参数

- setClampingThresMax/setClampingThresMin：这两个函数决定了一个体元执行 log-odd 更新的阈值范围。也就是说某一个占据体元的概率值爬升到 0.97（对应的log-odd为3.5）或者空闲体元的概率值下降到 0.12（对应的log-odd为-2）便不再进行 log-odd 更新计算

- setOccupancyThres：这个函数定义了 octomap 判定某一个体元属于占据状态的阈值（isNodeOccupied函数），默认是 0.5，一般情况下我们将其设定为 0.7

- octomap::OcTreeNode::setLogOdds：设置节点的对数占有概率

- writeBinary(...)：存储地图

- bin/octovis：3D 可视化栅格地图工具

### 3.1 一些总结

log-odd 与概率值之间可以相互转化，因此在工程实现时 octomap 八叉树节点类 OcTreeNode 存储的数值是 log-odd 数值，并不是概率值。

## 四、Octomap 基本编程

### 4.1 建树

```cpp
// 这是最简单的 octomap，没有设置颜色，0.03 是分辨率可以自己更改，单位米
octomap::OcTree* octomap = new octomap::OcTree(0.03);

// 这是彩色 octomap，0.03 是分辨率可以自己更改，单位米
octomap::ColorOcTree* octomap = new octomap::ColorOcTree(0.03);
```

### 4.2 插入点

```cpp
// updateNode 是插入单个点，接受两个参数，第一个就是 octomap::point3d 类型的 3d 点
// 第二个参数默认填 true 即可
double x = 1;
double y = 2;
double z = 3;
octomap->updateNode(octomap::point3d(x, y, z), true);
```

### 4.3 插入射线

插入射线需要起始点和终点，在终点位置添加一个 node（或者已有node的概率升高），在起始点到终点之间经过的 node概率降低（可以认为起始点是观测点，终点是目标点，那么我既然观测到了目标点，说明目标点之前没有遮挡，如果有遮挡，可能是误差造成的，那就需要对这个遮挡点进行降权） - 复制 CSDN

```cpp
// insertRay 是插入射线，有两个参数，依次是起始点和终点，都是 octomap::point3d 类型的 3d 点
// 例子是以（0，0，0）为起点，（1，2，3）为终点的射线
double x = 1;
double y = 2;
double z = 3;
octomap->insertRay(octomap::point3d(0, 0, 0), octomap::point3d(x, y, z));
```

### 4.4 加上颜色

```cpp
// 给彩色 octomap 设定 RGB 颜色, 对应六个参数分别是目标点 xyz 颜色 RGB
unsigned int R = 216;
unsigned int G = 120;
unsigned int B = 255;
octomap->setNodeColor(x, y, z, R, G, B);
```

### 4.5 清空和剪枝

```cpp
//清空octomap
octomap->clear()
 
//剪枝（原理见高翔老师博客）
octomap->prune()
```

### 4.6 发布 Octomap Msg 主题

```cpp
// 声明 advertise，octomap rviz plugin 默认接受 topic 为 octomap_full 的 message
pub_octomap = n.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);

// 声明 message
// Octomap map_msg; 是不是写错了 ?
octomap_msgs map_msg;

// 设置 header
map_msg.header.frame_id = 'world';
map_msg.header.stamp = ros::Time::now();

// fullMapToMsg 负责转换成 message
if (octomap_msgs::fullMapToMsg(*octomap, map_msg))
    // 转换成功，可以发布了
    pub_octomap.publish(map_msg);
else
    ROS_ERROR("Error serializing OctoMap");
```

## 五、编程使用 Octomap

### 3.1 pcd 点云转 Octomap





### 3.2 给 Octomap 加上颜色





### 3.3 Octomap 建图

Octomap 建图就是利用位姿信息将多帧 Octomap 地图拼接成一个全局地图。



### 3.4 占有率和概率更新





### 3.5 建图逻辑

获取了实时点云和 octomap 数据之后就可构建环境障碍物，实现运动规划：

1. 可先订阅点云话题得到点云数据，滤波分割后再发布
2. 再订阅滤波、分割后点云数据给 Octomap_server 节点，从而得到更加有效的 Octomap 环境地图。



## 参考博客

- [SLAM拾萃(1)：octomap](https://www.cnblogs.com/gaoxiang12/p/5041142.html)
- [https://wiki.ros.org/octomap](https://wiki.ros.org/octomap)
- [使用octomap_server将点云地图转化为八叉树地图和占据栅格地图](https://blog.csdn.net/sylin211/article/details/93743724)
- [Octomap 在ROS环境下实时显示](https://blog.csdn.net/crp997576280/article/details/74605766)
- [视觉SLAM笔记（64） 八叉树地图](https://blog.csdn.net/qq_32618327/article/details/103215769?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-2&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-2)
- [https://blog.csdn.net/DJ_Dreamaker/article/details/79834954](https://blog.csdn.net/DJ_Dreamaker/article/details/79834954)
- [https://blog.csdn.net/weixin_39123145/article/details/82219968](https://blog.csdn.net/weixin_39123145/article/details/82219968)

