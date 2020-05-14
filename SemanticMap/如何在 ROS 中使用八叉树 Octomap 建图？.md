# 如何在 ROS 中使用八叉树 Octomap 建图？

 ## 一、Octomap 安装

八叉树 Octomap 项目地址：

- [Octomap](https://octomap.github.io/)
- [http://octomap.github.io/octomap/doc/](http://octomap.github.io/octomap/doc/)

### 1.1 ROS Kinect 功能包 apt 完全安装（推荐）

所有要安装的 Octomap 组件如下：

```shell
sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-rviz-plugins ros-kinetic-octomap-server
```

**建议使用通配符全部安装，将 kinetic 替换为你的 ROS 版本名称**：

```shell
sudo apt-get install ros-kinetic-octomap*
```

**注意**：如果 ros-kinetic-octomap-rviz-plugins 使用过程中出现 rviz core dumped 或者 segmentation default 之类令人莫名奇妙的 rviz 程序崩溃问题，重新使用源码编译安装 octomap-rviz-plugins，源码地址：

```shell
https://github.com/OctoMap/octomap_rviz_plugins
```

### 1.2 Octomap 源码编译安装（独立安装，与 ROS 无关）

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

### 1.3 配置 CMakeList.txt

```cmake
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

add_executable(xxx xxx.cpp)
target_link_libraries(${OCTOMAP_LIBRARIES})
```

### 1.4 配置 package.xml

```xml
<build_depend>octomap</build_depend>
<run_depend>octomap</run_depend>
```



## 二、Octomap 项目建图

### 3.1 建图方法

1. 使用 octomap_server 节点建图：[Octomap 在ROS环境下实时显示](https://blog.csdn.net/crp997576280/article/details/74605766)、[octomap_server](http://wiki.ros.org/octomap_server)
2. 自己编写建图节点：[SLAM拾萃(1)：octomap](https://www.cnblogs.com/gaoxiang12/p/5041142.html)

### 3.2 增量式建图逻辑

Octomap 建图就是利用位姿信息将多帧 octomap 地图拼接成一个全局地图，对于实现增量式的 octomap 构建（也就是像 SLAM 构建点云一样，一边走一边生成全局的 octomap）。

有两种方法实现：

1. 第一种方法是获取每次 SLAM 计算得到的**当前时刻位姿和点云数据**，利用这个**位姿把当前时刻的点云旋转到世界坐标系下**，然后旋转后的点云数据发布给 octomap_server 节点，由于 octomap_server 本身具有维护地图的功能，它自己会去拼接八叉树地图，这可以省去很多事情。 
2. 第二种方法是使用 PCL 点云库自带的地图维护工具，把 octomap 只当做一个转换工具，每次都发布全局的点云地图给 octomap 节点（随着点云数据的增大会出现程序崩溃的现象），这种方法下你可以将 ORB SLAM 的关键帧生成点云然后一直发布更新后的点云，这个代码高博以前写过了，可在 github 找到，你将这个包编译到 ROS 上以后，再将这个算法生成的全局点云地图发布到 octomap_server 节点上，也就可以实现实时的 octomap 构建，再做导航什么的就方便了。 - 不太理解这里的前面 octomap 指的是什么，可能是 octomap 库，而不是 octomap_server

### 3.3 其他相关建图的问题

[评论有建图的回答](https://blog.csdn.net/sinat_38068956/article/details/84038069)

```
你好，请问怎么实时将相机的点云转化为八叉树

对于整体点云，先创建 octomap，然后对所有点调用 updateNode 方法即可将点云插入到 octomap 中，单帧点云同理。如果是想整合多帧，需要借助 slam 计算帧间变换矩阵，然后对第一帧外的点云进行投影。
```



```
获取了实时点云和 octomap 数据之后就可构建环境障碍物，实现运动规划：

1. 可先订阅点云话题得到点云数据，滤波分割后再发布
2. 再订阅滤波、分割后点云数据给 Octomap_server 节点，从而得到更加有效的 Octomap 环境地图。
```



### 3.4 占有率和概率更新

需要自己管理占据概率更新吗？





### 3.5 建图的重要细节

#### 3.5.1 体素网格分辨率

- 0.01  - 0.05 m
- 0.5 m 地图范围较大
- 0.2 m 地图范围较大



## 参考博客

- [SLAM拾萃(1)：octomap](https://www.cnblogs.com/gaoxiang12/p/5041142.html)
- [https://wiki.ros.org/octomap](https://wiki.ros.org/octomap)
- [使用octomap_server将点云地图转化为八叉树地图和占据栅格地图](https://blog.csdn.net/sylin211/article/details/93743724)
- [Octomap 在ROS环境下实时显示](https://blog.csdn.net/crp997576280/article/details/74605766)
- [视觉SLAM笔记（64） 八叉树地图](https://blog.csdn.net/qq_32618327/article/details/103215769?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-2&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromBaidu-2)
- [https://blog.csdn.net/DJ_Dreamaker/article/details/79834954](https://blog.csdn.net/DJ_Dreamaker/article/details/79834954)
- [https://blog.csdn.net/weixin_39123145/article/details/82219968](https://blog.csdn.net/weixin_39123145/article/details/82219968)
- [**octomap, slam, path planning: how does it all fit together?**](https://answers.ros.org/question/221092/octomap-slam-path-planning-how-does-it-all-fit-together/)
