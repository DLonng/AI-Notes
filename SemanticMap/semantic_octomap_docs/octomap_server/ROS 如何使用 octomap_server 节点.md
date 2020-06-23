## octomap_server 使用方法

## 一、节点参数

- `frame_id` (`string`, default: /map) 很重要！
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
  - 是否滤除斑点，暂时未测试效果

## 二、要求 TF 变换

```shell
sensor data frame -> /map (static world frame, changeable with parameter frame_id)
```

Required transform of sensor data into the global map frame if you do scan integration. This information needs to be available from an external SLAM or localization node.

源代码 [OctomapServer.cpp](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/src/OctomapServer.cpp)：

```cpp
	tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
```

非常重要：增量式地图的构建需要输入话题的传感器数据帧和全局地图帧之间的 TF 变换，这个 TF 变换需要从其他 SLAM 或者定位节点获取。

对于项目即需要：当前融合点云帧 [相机坐标系] -> [全局融合坐标系] 全局导航地图帧的 TF 变换！

- 当前融合点云帧的坐标系是：我的融合逻辑是从点云坐标系到相机坐标系，所以应该是相机坐标系吧？待实验确定！

- 全局导航地图帧的坐标系是：车体全局（融合）坐标系？

## 三、ColorOctomap 启用方法

第一步、修改头文件，打开注释切换地图类型：[OctomapServer.h](https://github.com/OctoMap/octomap_mapping/blob/kinetic-devel/octomap_server/include/octomap_server/OctomapServer.h)

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

2 个核心的八叉树生成函数 `insertCloudCallback` 和 `insertScan`：

```cpp
#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif

// NB: Only read and interpret color if it's an occupied node
#ifdef COLOR_OCTOMAP_SERVER 
        m_octree->averageNodeColor(it->x, it->y, it->z, /*r=*/it->r, /*g=*/it->g, /*b=*/it->b);
#endif
```

## 四、octomap_saver

保存压缩的二进制存储格式地图：

```shell
octomap_saver mapfile.bt
```

保存一个完整的概率八叉树地图：

```
octomap_saver -f mapfile.ot
```

## 五、导航方法

一是在定位或者导航的 launch 文件中添加导入 .bt 文件：

```xml
  <node pkg="octomap_server" type="octomap_server_loc" name="octomap_server" output="screen" args="/home/．．．/map/tree.bt">
    <!-- resolution in meters per pixel -->
    <param name="resolution" value="0.1" />
    <!-- name of the fixed frame, needs to be "map" for SLAM  camera_init (aloam)   -->
    <param name="frame_id" type="string" value="map" />
    <!-- max range / depth resolution of the kinect in meter -->
    <param name="sensor_model/max_range" value="50" />
    <param name="latch" value="true" />
    <!-- data source to integrate (PointCloud2), we modified octomap to support 3 point cloud inputs-->
    <param name="filter_ground" value="False" />
    <param name="filter_speckles" type="bool" value="true" />
    <param name="ground_filter/distance" type="double" value="0.2" />
    <param name="ground_filter/plane_distance" type="double" value="0.2" />
    <!--param name="occupancy_min_z" type="double" value="1.0" / -->
    <!--param name="occupancy_max_z" type="double" value="2.5" / -->
    <!-- max/min height for occupancy map, should be in meters -->
    <param name="pointcloud_max_z" value="100" />
    <param name="pointcloud_min_z" value="-100" />

     <!-- max/min reload 0.12 0.97 -->
    <param name="sensor_model_min" value="0.12" />
    <param name="sensor_model_max" value="0.97" />
    <!-- miss/hit reload 0.4  0.7-->
    <param name="sensor_model_miss" value="0.4" />
    <param name="sensor_model_hit" value="0.7" />
```

二是在定位或者导航过程中 octomap-server 的 octomap_server.cpp 中添加.bt 文件：（这样有个好处就是，栅格地图是实时导入更新的，不会在导航时进行累积）：

```cpp
OcTree* octree = new OcTree("/home/．．．/map/tree.bt");
m_octree = octree
```

参考博客：

- https://blog.csdn.net/Discoverhfub/article/details/99584677