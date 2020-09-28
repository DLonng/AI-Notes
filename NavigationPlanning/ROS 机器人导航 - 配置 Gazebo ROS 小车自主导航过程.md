# ROS 配置 Gazabo 自主导航

## 一、安装导航包

### 1.1 Apt 安装：

这种适合不需要修改源码的同学：

```shell
sudo apt install ros-kinetic-navigation
```

### 1.2 源码编译安装

如果做研究，则推荐这种，因为可能自己要修改算法，先使用下面的方法安装依赖：

```shell
# 编译过程可能会遇到找不到 Bullet 等依赖问题，解决这些依赖简便办法
# 先 apt-get 安装 ros-navigation 包，这样依赖会被自动装上 
sudo apt-get install ros-kinetic-navigation* 
# 再 apt-get 卸载掉 ros-navigation 包 
sudo apt-get remove ros-kinetic-navigation ros-kinetic-navigation-experimental
```

然后从 Github 下载源码有 2 种方法，建议第二种也要掌握，比较方便：

#### 1.2.1 直接下载 zip 包

如果对 git 不熟悉，可以从这个 [navigation-kinetic-devel](https://github.com/ros-planning/navigation/tree/kinetic-devel) 分支直接下载 zip 代码到本地，然后放到工作空间编译即可，不会出现错误。

#### 1.2.2 git clone 代码

使用 git 命令直接下载全部代码到工作空间，然后切换到自己的 ros 版本分支：

```shell
git clone https://github.com/ros-planning/navigation.git
```

查看当前版本分支：

```shell
git branch -vv
```

```shell
*melodic-devel 4dca437 [origin/melodic-devel] Fix #796 (#1017) 
 kinetic-devel 3c4d04e [origin/kinetic-devel] Fix #796 (#1017)
```

当前是 melodic 版本，而我用的 kinetic，所以需要切换到 kinetic-devel 版本分支：

```shell
git checkout kinetic-devel
```

再次查看版本即可看到当前已经切换到 kinetic 版本：

```shell
* kinetic-devel 3c4d04e [origin/kinetic-devel] Fix #796 (#1017)
  melodic-devel 4dca437 [origin/melodic-devel] Fix #796 (#1017)
```

同样在工作空间 catkin_make 编译即可，不会出错。

### 1.3 安装局部规划器

kinetic 默认的局部规划器比较老，可以使用这个比较新的，编译安装后即可使用：

```shell
https://github.com/rst-tu-dortmund/teb_local_planner.git
```

依然是选择 kinetic-devel 下载 zip 包，拷贝到工作空间先安装依赖：

```shell
rosdep install teb_local_planner
```

之后 catkin_make 不会出错。

## 二、配置自主导航

### 2.1 基本步骤

1. Gazebo 对小车建模
2. 获取传感器数据
3. 获取 odom tf 数据
4. 获取 tf 定位
5. 可以提前构建地图
6. 配置 move_base 节点
7. 如果使用已经构建的地图，则配置 AMCL 节点
8. 如果一边定位一边导航，则需要启动 SLAM 节点（需要在我的电脑上配置 lego_loam）
9. 在 Gazebo 中测试
10. 在实际的小车上测试

注意 move_base 订阅的话题名是固定的：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/move_base.png)

### 2.2 配置过程

#### 2.2.1 创建导航包

agilex_navigation 用来存储导航用的配置文件和脚本，在内部创建如下文件：

#### 2.2.2 配置 AMCL 参数

3D 雷达能够使用 AMCL 吗？

#### 2.2.3 安装 pointcloud_to_laser

AMCL 需要 2D LaserScan 作为输入话题，所以用这个包把点云转为 laserscan：

```shell
sudo apt install ros-kinetic-pointcloud-to-laserscan
```

使用的 launch 参数还没调好，但是可以使用：

```xml
<?xml version="1.0"?>

<launch>

    <!-- run pointcloud_to_laserscan node -->
    <node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="pointcloud_to_laserscan">

        <remap from="cloud_in" to="/rslidar_points"/>
        <remap from="scan" to="/scan"/>
        <rosparam>
            target_frame: rslidar # Leave disabled to output scan in pointcloud frame
            transform_tolerance: 0.01
            min_height: 0.0
            max_height: 1.0

            angle_min: -3.14 #-1.5708 # -M_PI/2
            angle_max: 3.14  #1.5708 # M_PI/2
            angle_increment: 0.0087 # M_PI/360.0
            scan_time: 0.3333
            range_min: 0.45
            range_max: 10.0
            use_inf: true

            # Concurrency level, affects number of pointclouds queued for processing and number of threads used
            # 0 : Detect number of cores
            # 1 : Single threaded
            # 2->inf : Parallelism level
            concurrency_level: 1
        </rosparam>

    </node>

</launch>

```

参考链接：

- [Using PointCloud in AMCL [closed]](https://answers.ros.org/question/196010/using-pointcloud-in-amcl/)
- [pointcloud_to_laserscan](http://wiki.ros.org/pointcloud_to_laserscan)

#### 2.2.4 配置 move_base 参数

配置导航参考的博客，主要看官网文档：

- https://www.guyuehome.com/8063
- https://www.guyuehome.com/281
- http://wiki.ros.org/navigation

#### 2.2.5 使用 [map_server](http://wiki.ros.org/map_server)

map_server 用来给 move_base 提供导航用的 2D 网格地图 `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))

#### 2.2.5 安装 lego_loam

主要是为了模拟小车的环境，安装方法在官方项目：[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)

## 三、自主导航实验

### 3.1 实验一：Gazebo - 地图导航

#### 3.1.1 从八叉树构建 2D 网格地图

先要配置 `octomap_server` 滤除地面网格，否则投影后没有地面不能导航，这里使用小车的轮式里程计的 `/odom` 作为地图坐标系：

```xml
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">

    <!-- resolution in meters per pixel -->
    <param name = "resolution" value = "0.10" />

    <!-- name of the fixed frame, needs to be "/map" for SLAM -->
    <!-- 静态全局地图的 frame_id，但在增量式构建地图时，需要提供输入的点云帧和静态全局帧之间的 TF 变换 -->
    <param name = "frame_id" type = "string" value = "odom" />

    <!-- set min to speed up! -->
    <param name = "sensor_model/max_range" value = "15.0" />

    <!-- 机器人坐标系 base_link，滤除地面需要该 frame -->
    <param name = "base_frame_id" type = "string" value = "base_link" />

    <!-- filter ground plane, distance value should be big! 项目并不需要滤除地面 -->
	<param name = "filter_ground" type = "bool" value = "true" />
    <param name = "ground_filter/distance" type = "double" value = "0.1" />
    <!-- 分割地面的 Z 轴阈值 value 值 -->
	<param name = "ground_filter/plane_distance" type = "double" value = "0.1" />

    <!-- 直通滤波的 Z 轴范围，保留 [-1.0, 10.0] 范围内的点 -->
    <!-- <param name = "pointcloud_max_z" type = "double" value = "100.0" /> -->
    <!-- <param name = "pointcloud_min_z" type = "double" value = "-1.0" /> -->

    <!-- <param name = "filter_speckles" type = "bool" value = "true" /> -->

    <param name = "height_map" value = "false" />
    <param name = "colored_map" value = "false" />

    <param name = "outrem_radius" type = "double" value = "1.0" /> 
    <param name = "outrem_neighbors" type = "int" value = "10" /> 

    <!-- when building map, set to false! -->
    <param name = "latch" value = "false" /> 

    <!-- topic from where pointcloud2 messages are subscribed -->
    <!-- 要订阅的点云主题名称 /pointcloud/output -->
    <!-- 这句话的意思是把当前节点订阅的主题名称从 cloud_in 变为 /pointcloud/output -->
    <remap from = "/cloud_in" to = "/semantic_cloud_max" />
 
  </node>
</launch>
```

使用 octomap_server 建图包可以得到 2D 网格地图，保存即可：

```shell
rosrun map_server map_saver -f map_name map:=/projected_map
```

同时保存八叉树地图：

```shell
octomap_saver -f octomap_name.ot
```

**！！！需要完成的工作**：给 `octomap_generator` 增加过滤地面和投影 2D 网格地图的功能！

#### 3.1.2 加载地图并导航

启动的时候加载 2D 网格地图，使用 AMCL 来获取全局的机器人位置：

```xml
<launch>

    <!-- 读取已经构建的地图，设置地图的配置文件，我应该要把 octomap 转为这种地图类型 -->
    <arg name="map" default="mini_gazebo_map2.yaml" />

    <!-- 运行地图服务器，并且加载设置的地图，发布 /map-->
    <node name="map_server" pkg="map_server" type="map_server" args="$(find agilex_navigation)/maps/$(arg map)"/>

    <!-- 运行 point_to_laser 节点 -->
    <include file="$(find agilex_navigation)/launch/agilex_pointcloud_to_laserscan.launch"/>

    <!-- 运行 move_base 节点 -->
    <include file="$(find agilex_navigation)/launch/agilex_mini_move_base.launch"/>

    <!-- 启动 AMCL 节点 -->
    <include file="$(find agilex_navigation)/launch/agilex_mini_amcl.launch" />

    <!-- 对于虚拟定位，需要设置一个 /odom 与 /map 之间的静态坐标变换 -->
    <node pkg="tf" type="static_transform_publisher" name="map_odom_broadcaster" args="0 0 0 0 0 0 /map /odom 100" />

    <!-- 运行 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find agilex_navigation)/rviz/agilex_mini_nav.rviz"/>

</launch>
```

实验结果：可以实验从 A 点走到 B 点。

### 3.2 实验二：Gazebo -  odom 导航

可以实现导航，但是只能选择雷达范围内的导航目标点。

### 3.5 实验三：Gazebo -  loam 导航

会漂，不能正确导航。



#### 3.4 实验四：Gazebo - loam 导航

把 odom 换成 camera_init，并禁止小车发送 odom。

tf 关系：

- map -> camera_init
- camera -> base_footprint

```xml
    <node pkg="tf" type="static_transform_publisher" name="camera_init_to_map"  args="0 0 0 1.570795   0        1.570795 /map /camera_init 10" />
    <node pkg="tf" type="static_transform_publisher" name="base_link_to_camera" args="0 0 0 -1.570795 -1.570795 0 /camera /base_footprint 10" />
```

代价地图配置：

- global：/map
- local：/camera_init

move_base launch 修改：

```xml
<remap from="odom" to="camera_init"/>
```

可以导航，但是会漂！

### 3.4 实验四：小车测试 - odom 导航



### 3.x 实验 X：小车测试 - 使用导航地图



### 3.x 实验 X：小车测试 - SLAM + 导航





### 3.x 实验二：Gazebo -  gmapping 导航

重启 roscore：

```shell
roscore
```

启动 gazebo 仿真：

```shell
roslaunch agilex_gazebo start_agilex_mini_gazebo.launch
```

启动 mbot 包里面的 gmapping：

```shell
roslaunch mbot_navigation gmapping.launch
```

启动 mini_nav 导航：

```shell
roslaunch agilex_navigation start_agilex_mini_nav_with_gmapping.launch
```

## 四、详细配置导航参数文件

### 4.1 teb_local_planner

- https://www.knightdusk.cn/2019/06/features-and-tuning-guide-for-teb-local-planner/
- https://www.guyuehome.com/9811
- [https://blog.csdn.net/weixin_44917390/article/details/107568507#TEB%20%E7%9A%84%E5%8F%82%E6%95%B0%E8%B0%83%E8%AF%95%EF%BC%9A](https://blog.csdn.net/weixin_44917390/article/details/107568507#TEB 的参数调试：)

### 4.2 dwa_local_planner

- 倾向于设定最大平移速度和旋转速度低于它们的实际值
- 对于最小平移速度，我们将它设置为一个大的负值，因为这将可以使机器人陷入困境中时可以后退，即使大多数情况下它都是前进的
- 对于最小的旋转速度，如果参数允许，我们将其设置为负值，以便机器人可以在任意方向旋转。要注意，DWA 本地规划器采用的是机器人最小旋转速度的绝对值
- x 方向的速度是指平行于机器人直线运动方向的速度，这与移动速度相同，y 方向的速度垂直于直线运动，它被称之为「冲击速度」，对于非整体机器人（如差速轮机器人），y 速度应该设置为零
- carrot 规划器检查给定的目标点是否是障碍物，如果是，则通过沿机器人和目标点的向量来选择靠近原始目标的替代目标。最终他将给局部规划器或内部控制器一个有效的目标点。因此，这种规划器没有任何全局路径规划
- navfn 使用 dijkstra 算法来在起点和终点之间寻找最小代价路线。global 规划器建立了更灵活的替代 navfn 的选择，这些选择包括：（1）支持 A * 算法（2）切换二次近似（3）切换网格路径。
- 仿真时间越长，计算负荷越大。此外，当仿真时间变长后，本地路径规划器产生路径的时间也会变长，这是合理的。如果将仿真时间设置为非常低的值（≤2.0）将导致性能有限，特别是当机器人需要通过狭窄的门口或家具之间的间隙时，因为没有足够的时间来获得最佳轨迹来通过狭窄的通道。另一方面，由于使用了 DWA 本地规划器，所有的轨迹都是简单的圆弧，如果将仿真时间设置的非常高（≥5.0），将导致长曲线不是非常灵活。
- vx_sample，vy_sample 确定在 x，y 方向上取多少平移速度样本。vth_sample 控制旋转速度样本的数量。样本的数量取决于你的计算能力。在大多数情况下，我们倾向于设置 vth_sample 高于平移速度样本，因为通常旋转比直线前进更复杂。如果将 y 向最大速度设置为零，则没必要在 y 方向提取速度样本，因为没有可用的样本。我们设置 vx_sample=20,vth_samples=40
- 仿真粒度：sim_granularity 是在轨迹上的点之间采取的步长。它意味着要多频繁的检查轨迹上的点（检测它们是否与障碍物相交）。较低的值意味着高频率，这需要更多的计算能力
- 目标函数的值依赖于三个组成部分：到目标点的过程、清除障碍物和前进速度。
- 目标是获得最小的代价，path_distance_bias 是本地规划器与全局路径保持一致的权重，较大的值将使本地规划器更倾向于跟踪全局路径
- goal_distance_bias 是机器人尝试到达目标点的权重，实验显示增加 goal_distance_bias 值将会使机器人与全局路径的一致性偏低
- occdist_scale 是机器人尝试躲避障碍物的权重，这个值偏大将导致机器人陷入困境。
- 在 RO S 中，代价地图由静态地图层、障碍物图层和膨胀层组成。静态地图层直接给导航堆栈提供静态 SLAM 地图解释，障碍物图层包含 2D 障碍物和 3D 障碍物（体素层）
- 膨胀层是将障碍物膨胀来计算每个 2D 代价地图单元的代价。
- 全局代价地图是通过膨胀导航堆栈上的地图障碍物来实现的，局部代价地图是通过将机器人传感器检测到的障碍物膨胀产生的。
- 足迹是移动基站的轮廓。在 ROS 中，它由二维数组表示 [x0,y0] ; [x1,y1] ; [x2,y2]…… 不需要重复第一个坐标。该占位面积将用于计算内切圆和外接圆的半径，用于以适合此机器人的方式对障碍物进行膨胀。为了安全起见，我们通常将足迹稍大于机器人的实际轮廓。
- inflation_radius 和 cost_scaling_factor 是决定膨胀的主要参数。inflation_radius 控制零成本点距离障碍物有多远。cost_scaling_factor 与单元的代价值成反比，设置高值将使衰减更陡峭
- 最佳的代价图衰减曲线是具有相对较低斜率的曲线，以便最佳路径尽可能远离每侧的障碍物，优点是机器人可以在障碍物中间移动
- 代价地图精度 costmap resolution，它们影响计算负荷和路径规划能力。在低分辨率率（≥0.05）的情况下，障碍物区域可能重叠，导致本地规划器无法找到可用路径
- 对于全局代价地图精度，只要保持与提供给导航堆栈的地图的分辨率相同即可
- 障碍物层跟踪二维的，体素层跟踪三维的。障碍物是根据机器人传感器的数据进行标记（检测）或清除（删除），其中需要订阅代价图的主题。
- 体素层从障碍物层继承，并且都是通过使用激光雷达发布的 Point Cloud 或 PointCloud2 类型的消息来获取障碍物信息，3D 障碍物最终会被膨胀为二维代价图
- 体素网格是一个 ROS 包，它提供了一个高效的三维体素网格数据结构的实现，它存储三种状态的体素：标记、自由、未知。体素网格占据了代价地图区域内的体积。在每次更新体素边界期间，体素层根据传感器的数据来标记或去除体素网格中的一些体素。它还执行光线跟踪，接下来会讨论。请注意，在更新时，不会重新创建体素网格，而仅仅在更改本地代价图的大小时才更新。
- max_obstacle_height：插入代价图中的障碍物的最大高度。该参数设置为稍高于机器人的高度，对于体素层，这基本上是体素网格的高度。
- obstacle_range：障碍物距离机器人的最大距离，障碍物以米为单位掺入代价地图，并可以在每个传感器的基础上进行覆盖
- raytrace_range：用于使用传感器数据在地图中扫描出障碍物，以米为单位，可以在每个传感器的基础上进行覆盖
- ROS 导航包有两种恢复行为，分别是清除代价地图恢复和旋转恢复。清除代价地图恢复是将本地代价地图还原成全局代价地图的状态，旋转恢复是通过旋转 360° 来恢复。
- ROS 恢复行为的参数一般设为默认值

参考博客：

- https://www.cnblogs.com/cv-pr/p/5800270.html
- https://zhuanlan.zhihu.com/p/143202720
- https://blog.csdn.net/heyijia0327/article/details/44983551?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-2.channel_param
- https://blog.csdn.net/x_r_su/article/details/53089351?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.channel_param
- https://www.cnblogs.com/flyinggod/p/12610883.html

























## 未解决的问题

- move_base 是否需要 map_server 提供代价地图？
- move_base 订阅的点云是否可以是预处理后的点云？
- 自己 octomap_server 投影的 costmap 和 move_base 内部通过点云生成的 costmap 有无区别？
- 自己导航是否应该先构建 2D 导航地图？
- 小车的 TF 树是否要包含 4 个轮子？





## X、遇到的问题

- 一边 SLAM 一边导航还需要使用 AMCL 吗？不需要，一边定位一边导航只需要使用 SLAM 和 move_base 即可
- `/odom` 从哪里订阅？
- `/odom` 的 tf 和 SLAM 定位的 tf 有和不同？

#### 1 Unable to get starting pose of robot, unable to create global plan

增大 transform_tolerance 参数：

```xml
global_costmap:
   global_frame: /map
   robot_base_frame: base_link
   update_frequency: 1.0
   publish_frequency: 1.0
   static_map: true
   rolling_window: false
   resolution: 0.01
   transform_tolerance: 3.0
   map_type: costmap
```

参考链接：

- https://answers.ros.org/question/58796/move_base-warning-unable-to-get-starting-pose-of-robot-unable-to-create-global-plan/

#### 2 TF 中不要存在单独的帧节点

把 gazebo 中 odom 去掉，lab3 的导航不会报 Unable to get starting pose of robot, unable to create global plan 错了。

#### 3 roslaunch 启动失败

偶然遇到 octomap_generator 启动失败：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/ros_error_process_died.png)

头文件产生的冲突，我把头文件全部放到 .h 文件中即可，按照系统、ros、pcl、自己头文件的顺序：

```cpp
#include <ros/ros.h>

#include <iostream>
#include <sstream>
#include <string>

#include <cmath>
#include <cstring>
#include <memory>

#include <boost/shared_ptr.hpp>

#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <pcl/filters/radius_outlier_removal.h>

// 更改了头文件的位置，节点启动失败的问题就解决了，可能是头文件包含有冲突
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>

// 编译完消息后，如果没有找到头文件
// 重新 source，然后重启 code
#include <scout_msgs/ScoutStatus.h>
#include <octomap_generator/octomap_generator.h>
#include <semantics_octree/semantics_octree.h>
```

具体冲突的地方暂时不确定，以后可以调试查看。

#### 4 Ubuntu 的 PCL 版本

系统安装了 3 个 PCL，1.8 自己装的，1.7 ros 装的：

- `/usr/local/include/pcl-1.8/pcl`、`/usr/local/lib/libpcl_*`
- `/usr/include/pcl-1.7/pcl/`、`/usr/lib/x86_64-linux-gnu/libpcl_*`
- `/opt/ros/kinetic/include/pcl_ros/`

分割链接错误的原因可能跟 PCL 版本有关，在小车上测试，CMakeLists 中 PCL 也要加上应该。

#### 5 RVIZ 机器人初始位置

```shell
The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?
```

解决方法：https://answers.ros.org/question/66399/move_base-start-position-is-off-the-global-costmap/

#### 6 NO Path

```shell
[ERROR] [1599035118.458805405, 428.072000000]: NO PATH!
[ERROR] [1599035118.458838884, 428.072000000]: Failed to get a plan from potential when a legal potential was found. This shouldn't happen.
```

#### 7 小车到达终点原地打转

局部路径规划器参数没调好



参考博客：

- https://blog.csdn.net/Enterenvy/article/details/90699274

#### 8 小车走的不平滑

路径规划器参数没有调好

#### 9 ros tf has two or more unconnected tree

取消多余的 odom，比如我取消了小车底盘的 odom tf

#### 10 DWA fail to product path

#### 11 The origin for the sensor at (0.26, -0.00) is out of map bounds. So, the costmap cannot raytrace for it

