构建语义地图时，最开始用的是 [octomap_server](https://github.com/OctoMap/octomap_mapping)，后面换成了 [semantic_slam: octomap_generator](https://github.com/floatlazer/semantic_slam)，不过还是整理下之前的学习笔记。

## 一、增量构建八叉树地图步骤

为了能够让 octomap_server 建图包实现增量式的地图构建，需要以下 2 个步骤：

### 1.1 配置 launch 启动参数

这 3 个参数是建图必备：

- 地图分辨率 `resolution`：用来初始化地图对象
- 全局坐标系 `frame_id`：构建的全局地图的坐标系
- 输入点云话题 `/cloud_in`：作为建图的数据输入，建图包是把一帧一帧的点云叠加到全局坐标系实现建图

```xml
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    <!-- resolution in meters per pixel -->
    <param name="resolution" value="0.10" />

    <!-- 增量式构建地图时，需要提供输入的点云帧和静态全局帧之间的 TF 变换 -->
    <param name="frame_id" type="string" value="map" />

    <!-- 要订阅的点云主题名称 /fusion_cloud -->
    <remap from="/cloud_in" to="/fusion_cloud" />
  </node>
</launch>
```

以下是所有可以配置的参数：

- `frame_id` (`string`, default: /map) 
  - Static global frame in which the map will be published. A transform from sensor data to this frame needs to be available when dynamically building maps.
- `resolution` (`float`, default: 0.05)
  - Resolution in meter for the map when starting with an empty map. Otherwise the loaded file's resolution is used
- `height_map` (`bool`, default: true)
  - Whether visualization should encode height with different colors
- `color/[r/g/b/a]` (`float`)
  - Color for visualizing occupied cells when ~heigh_map=False, in range [0:1]
- `sensor_model/max_range` (`float`, default: -1 (unlimited))
  - 动态构建地图时用于插入点云数据的最大范围（以米为单位），将范围限制在有用的范围内（例如5m）可以防止虚假的错误点。
- `sensor_model/[hit|miss]` (`float`, default: 0.7 / 0.4)
  - 动态构建地图时传感器模型的命中率和未命中率
- `sensor_model/[min|max]` (`float`, default: 0.12 / 0.97)
  - 动态构建地图时的最小和最大 clamp 概率
- `latch` (`bool`, default: True for a static map, false if no initial map is given)
  - 不管主题是锁定发布还是每次更改仅发布一次，为了在构建地图（频繁更新）时获得最佳性能，请将其设置为 false，如果设置为 true，在每个地图上更改都会创建所有主题和可视化。
- `base_frame_id`(string, default: base_footprint)
  - The robot's base frame in which ground plane detection is performed (if enabled)
- `filter_ground`(bool, default: false)
  - 动态构建地图时是否应从扫描数据中检测并忽略地平面，这会将清除地面所有内容，但不会将地面作为障碍物插入到地图中。如果启用此功能，则可以使用 ground_filter 对其进行进一步配置
- `ground_filter/distance` (`float`, default: 0.04)
  - 将点（在 z 方向上）分割为接地平面的距离阈值，小于该阈值被认为是平面
- `ground_filter/angle` (`float`, default: 0.15)
  - 被检测平面相对于水平面的角度阈值，将其检测为地面
- `ground_filter/plane_distance` (`float`, default: 0.07)
  - 对于要检测为平面的平面，从 z = 0 到距离阈值（来自PCL的平面方程的第4个系数）
- `pointcloud_[min|max]_z` (`float`, default: -/+ infinity)
  - 要在回调中插入的点的最小和最大高度，在运行任何插入或接地平面滤波之前，将丢弃此间隔之外的任何点。您可以以此为基础根据高度进行粗略过滤，但是如果启用了 ground_filter，则此间隔需要包括接地平面。
- `occupancy_[min|max]_z` (`float`, default: -/+ infinity)
  - 最终 map 中要考虑的最小和最大占用单元格高度，发送可视化效果和碰撞 map 时，这会忽略区间之外的所有已占用体素，但不会影响实际的 octomap 表示。
- `filter_speckles`(bool)
  - 是否滤除斑

### 1.2 要求 TF 变换

有了全局坐标系和每一帧的点云，但是建图包怎么知道把每一帧点云插入到哪个位置呢？

因为随着机器人的不断移动，会不断产生新的点云帧，每个点云帧在全局坐标系中插入的时候都有一个确定的位置，否则构建的地图就不对了，因此需要给建图包提供一个当前帧点云到全局坐标系的位姿，这样建图包才能根据这个位姿把当前获得的点云插入到正确的位置上。

在 ROS 中可以很方便的使用 TF 来表示这个变换，我们只需要在启动建图包的时候，在系统的 TF 树中提供「cloud frame -> world frame」的变换就可以了：

```shell
cloud frame -> world frame (static world frame, changeable with parameter frame_id)
```

注意：

- `cloud frame`：就是输入点云话题中 `head.frame_id`，比如 Robosense 雷达的 `frame_id = rslidar`
- `world frame`：这是全局坐标系的 frame_id，在启动 launch 中需要手动给定，比如我给的是 `map`

如果你给 `world frame id` 指定的是输入点云的 `frame_id`，比如 `fusion_cloud.head.frame_id == wolrd_frame_id == rslidar`，则只会显示当前帧的八叉树，而不会增量构建地图，这点要注意了，可以自己测试看看。

那么为了增量式建图，还需要在系统的 TF 树中提供「rslidar -> world」的变换，这个变换可以通过其他的 SLAM 等获得，比如我测试时候的一个 TF 树如下：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/octomap_server_tf_test.png)

我找了下源代码 [OctomapServer.cpp](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp) 中寻找 TF 的部分：

```cpp
	tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
```

总体来说这个建图包使用起来还是挺简单的，最重要的就是要写清楚输入点云话题和 TF 变换。

#### 小 Tips：没有 TF 怎么办？

我刚开始建图的时候，我同学录制的 TF 树中并没有「world -> rslidar」的变换，只有「world -> base_link」，所以为了能够测试增量式建图，因为我的点云帧的 frame_id 是 rslidar，因此我就手动发布了一个静态的「base_link -> rslidar」的变换：

```shell
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar
```

这样系统中就有「rslidar -> world」的变换了，但是我发的位姿都是 0，所以对建图测试没有影响，为了方便启动，放在 launch 中：

```shell
<node pkg = "tf2_ros" type = "static_transform_publisher" name = "dlonng_static_test_broadcaster" args = "0 0 0 0 0 0 base_link rslidar" />
```

如果你也遇到这个问题，可以试试发个静态 TF 做做测试，关于静态 TF 详细技术可以参考之前的文章：[ROS 机器人技术 - 静态 TF 坐标帧](https://dlonng.com/posts/static-tf)

## 二、ColorOctomap 启用方法

为了显示 RGB 颜色，我分析了下源码，第一步修改头文件，打开注释切换地图类型：[OctomapServer.h](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/include/octomap_server/OctomapServer.h)

```cpp
// switch color here - easier maintenance, only maintain OctomapServer. 
// Two targets are defined in the cmake, octomap_server_color and octomap_server. One has this defined, and the other doesn't
// 打开这个注释
#define COLOR_OCTOMAP_SERVER

#ifdef COLOR_OCTOMAP_SERVER
  typedef pcl::PointXYZRGB PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;
#endif
```

[CMakeList.txt](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/CMakeLists.txt) 文件中有 `COLOR_OCTOMAP_SERVER` 的编译选项：

```cpp
target_compile_definitions(${PROJECT_NAME}_color PUBLIC COLOR_OCTOMAP_SERVER)
```

[OctomapServer.cpp](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp) 中有 colored_map 的参数：

```cpp
m_useHeightMap = true;
m_useColoredMap = false;
  
m_nh_private.param("height_map", m_useHeightMap, m_useHeightMap);
m_nh_private.param("colored_map", m_useColoredMap, m_useColoredMap);
```

地图默认是按照高度设置颜色，如果要设置为带颜色的地图，就要禁用 HeightMap，并启用 ColoredMap：

```cpp
if (m_useHeightMap && m_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    m_useColoredMap = false;
  }
```

第二步、需要在 octomap_server 的 launch 文件中禁用 height_map，并启用 [colored_map](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp) ：

```xml
<param name="height_map" value="false" />
<param name="colored_map" value="true" />
```

2 个核心的八叉树生成函数 `insertCloudCallback` 和 `insertScan` 中有对颜色的操作：

```cpp
#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif

// NB: Only read and interpret color if it's an occupied node
#ifdef COLOR_OCTOMAP_SERVER 
        m_octree->averageNodeColor(it->x, it->y, it->z, /*r=*/it->r, /*g=*/it->g, /*b=*/it->b);
#endif
```

## 三、保存和显示地图

启动 octomap_server 节点后，可以使用它提供的地图保存服务，保存压缩的二进制存储格式地图：

```shell
octomap_saver mapfile.bt
```

保存一个完整的概率八叉树地图：

```
octomap_saver -f mapfile.ot
```

安装八叉树可视化程序 octovis 来查看地图：

```shell
sudo apt-get install octovis
```

安装后重启终端，使用以下命令显示一个八叉树地图：

```shell
octovis xxx.ot[bt]
```

## 四、源码阅读笔记

在开组会汇报的时候，整理了以下这个建图包的关键流程，只有 2 个关键的函数也不是很复杂，我给代码加了注释，在文末可以下载。

第一步是订阅点云的回调：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/insert_pointcloud.png)

第二步是插入单帧点云构建八叉树，这里就不详细介绍过程了，因为涉及到八叉树库 octomap 的更新原理：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/insert_scan.png)

放一张我们学院后面的一条小路的建图结果，分辨率是 15cm：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/map_test.png)

以下是我建图过程的 launch：

```xml
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
    <!-- resolution in meters per pixel -->
    <param name = "resolution" value = "0.15" />

    <!-- name of the fixed frame, needs to be "/map" for SLAM -->
    <!-- 静态全局地图的 frame_id，但在增量式构建地图时，需要提供输入的点云帧和静态全局帧之间的 TF 变换 -->
    <param name = "frame_id" type = "string" value = "camera_init" />

    <!-- set min to speed up! -->
    <param name = "sensor_model/max_range" value = "15.0" />

    <!-- 机器人坐标系 base_link，滤除地面需要该 frame -->
    <!-- <param name = "base_frame_id" type = "string" value = "base_link" /> -->

    <!-- filter ground plane, distance value should be big! 项目并不需要滤除地面 -->
	<!-- <param name = "filter_ground" type = "bool" value = "true" /> -->
    <!-- <param name = "ground_filter/distance" type = "double" value = "1.0" /> -->
    <!-- 分割地面的 Z 轴阈值 value 值 -->
	<!-- <param name = "ground_filter/plane_distance" type = "double" value = "0.3" /> -->

    <!-- 直通滤波的 Z 轴范围，保留 [-1.0, 10.0] 范围内的点 -->
    <!-- <param name = "pointcloud_max_z" type = "double" value = "100.0" /> -->
    <!-- <param name = "pointcloud_min_z" type = "double" value = "-1.0" /> -->

    <!-- <param name = "filter_speckles" type = "bool" value = "true" /> -->

    <param name = "height_map" value = "false" />
    <param name = "colored_map" value = "true" />
	
    <!-- 增加了半径滤波器 -->
    <param name = "outrem_radius" type = "double" value = "1.0" />
    <param name = "outrem_neighbors" type = "int" value = "10" />

    <!-- when building map, set to false to speed up!!! -->
    <param name = "latch" value = "false" /> 

    <!-- topic from where pointcloud2 messages are subscribed -->
    <!-- 要订阅的点云主题名称 /pointcloud/output -->
    <!-- 这句话的意思是把当前节点订阅的主题名称从 cloud_in 变为 /pointcloud/output -->
    <remap from = "/cloud_in" to = "/fusion_cloud" />
 
  </node>
</launch>
```

我做的项目代码在这里：[AI - Notes: semantic_map](https://github.com/DLonng/AI-Notes/tree/master/SemanticMap/semantic_map_ws)