## 导航方法

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