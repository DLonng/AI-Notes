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
  - Maximum range in meter for inserting point cloud data when dynamically building a map. Limiting the range to something useful (e.g. 5m) prevents spurious erroneous points far away from the robot.

- `sensor_model/[hit|miss]` (`float`, default: 0.7 / 0.4)
  - Probabilities for hits and misses in the sensor model when dynamically building a map

- `sensor_model/[min|max]` (`float`, default: 0.12 / 0.97)
  - Minimum and maximum probability for clamping when dynamically building a map

- `latch` (`bool`, default: True for a static map, false if no initial map is given)
  - Whether topics are published latched or only once per change. For maximum performance when building a map (with frequent updates), set to false. When set to true, on every map change all topics and visualizations will be created.

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

