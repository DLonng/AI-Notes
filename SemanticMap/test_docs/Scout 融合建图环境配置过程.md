记录第一次在 scout 大车上编译运行融合，建图节点的过程。

## 一、标定

先使用小车的标定矩阵测试。

## 二、编译

创建完工作空间后，把每个包单独放入 src 中编译，同样需要安装 octomap 库：

```shell
sudo apt-get install ros-kinetic-octomap  ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-pa ros-kinetic-octomap-rviz-plugins
```

小车网卡和雷达有冲突，需要把雷达拔掉才能联网，编译不会有错误，但是 roslaunch 运行时提示如下错误：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/scout_roslaunch_error2.png)

查找了下资料，发现是因为工作目录含有中文「桌面」，于是我在其他的英文目录重新创建了工作空间，然后重新编译运行就没问题了，后面需要把小车的语言换成英文的（设置 -> 语言支持），防止以后同学再次遇到这个很坑的问题。

## 三、启动测试

### 3.1 要启动的节点

相机：如果不能启动，需要更换 USB 接口，并运行桌面的 `ZED_Explorer` 程序启动相机驱动

```
roslaunch zed_cpu_ros zed_cpu_ros.launch
```

雷达：直接以小车的 launch 启动

```shell
roslaunch rslidar_pointcloud rs_lidar_16.launch
```

融合节点：

```shell
roslaunch lidar_camera_fusion lidar_left_camera_fusion.launch
```

建图节点：

```shell
roslaunch octomap_generator scout_octomap_generator.launch
```

loam 节点，与小车不同：

```shell
roslaunch lego_loam run_laser_odom.launch

roslaunch lego_loam demo.launch
```

tf2_ros 静态转换节点

```shell
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar
```

小车的节点（用于获取 ScoutStatus 话题）：

```shell
./init_can.sh

roslaunch scout_bringup scout_minimal.launch
```

硬件驱动单独启动的 launch：

```xml
<launch>
  <!-- Start ZED! -->
  <include file = "$(find zed_cpu_ros)/launch/zed_cpu_ros.launch" />

  <!-- Start Robosense LIDAR! -->
  <include file = "$(find rslidar_pointcloud)/launch/rs_lidar_16.launch" />
  
  <!-- Start scout, need run init_can.sh -->
  <include file = "$(find scout_bringup)/launch/scout_minimal.launch" /> 
</launch>
```

融合建图节点单独启动 launch：

```xml
<launch>
  <!-- Start fusion node! -->
  <include file = "$(find lidar_camera_fusion)/launch/lidar_left_camera_fusion.launch" />

  <!-- Start Octomap node! -->
  <include file = "$(find octomap_generator)/launch/scout_octomap_generator.launch" />

  <!-- Start Lego Loam! -->
  <include file = "$(find lego_loam)/launch/run_laser_odom.launch" />
  <!-- <include file = "$(find lego_loam)/launch/demo.launch" /> -->
  
  <!-- Only using in map build test! -->
  <node pkg = "tf2_ros" type = "static_transform_publisher" name = "dlonng_octomap_generator_static_broadcaster" args = "0 0 0 0 0 0 base_link rslidar" />

  <!-- Start octomap_debug.rviz -->
  <node pkg = "rviz" type = "rviz" name = "dlonng_octomap_generator_rviz" respawn = "false" output = "screen" args = "-d $(find octomap_generator)/rviz/octomap_generator_debug.rviz"/>
  
</launch>
```

总的启动 launch：

```xml
<launch>
  <!-- Start ZED! -->
  <include file = "$(find zed_cpu_ros)/launch/zed_cpu_ros.launch" />

  <!-- Start Robosense LIDAR! -->
  <include file = "$(find rslidar_pointcloud)/launch/rs_lidar_16.launch" />

  <!-- Start fusion node! -->
  <include file = "$(find lidar_camera_fusion)/launch/lidar_left_camera_fusion.launch" />

  <!-- Start Octomap node! -->
  <include file = "$(find octomap_generator)/launch/scout_octomap_generator.launch" />

  <!-- Start Lego Loam! -->
  <include file = "$(find lego_loam)/launch/run_laser_odom.launch" />
  <!-- <include file = "$(find lego_loam)/launch/demo.launch" /> -->
  
  <!-- Start scout, need run init_can.sh -->
  <include file = "$(find scout_bringup)/launch/scout_minimal.launch" />
  
  <!-- Only using in map build test! -->
  <node pkg = "tf2_ros" type = "static_transform_publisher" name = "dlonng_octomap_generator_static_broadcaster" args = "0 0 0 0 0 0 base_link rslidar" />

  <!-- Start octomap_debug.rviz -->
  <node pkg = "rviz" type = "rviz" name = "dlonng_octomap_generator_rviz" respawn = "false" output = "screen" args = "-d $(find octomap_generator)/rviz/octomap_generator_debug.rviz"/>
  
</launch>
```

### 3.2 测试局部地图用的配置

使用最大概率的语义点云，但是不使用语义图像，并且置信度固定为 0.8，这是因为车上没有语义分割节点，RVIZ 显示的时候按照高度设置颜色即可，只是为了测试局部地图更新：

```cpp
#define USING_SEMANTIC 0
```

使用真实的小车速度测试局部地图：

```cpp
#define TEST_VEL 0
```

### 3.3 录制数据

- 左图像
- 雷达
- lego loam tf
- 小车状态

```shell
rosbag record /camera/left/image_raw /rslidar_points /tf /scout_status
```

录制完，记得关掉查看 bag 信息的终端，否则 U 盘会弹出失败！在 Nvida 上跑语义分割后，只需要录制语义分割后新产生的话题即可，然后在自己的机器上同时回放原始包和语义分割录制的包。

## 四、测试结果

### 4.1 scout 硬件频率

- 相机：20 Hz
- 雷达：10 Hz

话题频率测试命令：

```shell
rostopic hz /topic_name
```

### 4.2 Max Fusion RGB 

- 语义分割：15 Hz
- 融合节点：10 Hz
- 建图节点：全局地图 5 Hz，局部地图 2.5 Hz

### 4.3 Max Fusion Semantic 

暂时没有用 semantic_msg 消息：

- 语义分割：13 Hz
- 融合节点：10 Hz
- 局部 + 全局建图节点建图节点：全局地图 2.5 Hz，局部地图 2.5 Hz
- 全局建图节点：7 - 10 Hz

### 4.4 Baye Fusion Semantic 

目前只使用了 2 组语义：

- 语义分割：5 Hz
- 融合节点：10 Hz，在点云回调里面做的融合，所以频率会比语义分割快，后期要考虑把融合逻辑放到最慢的回调函数中
- 局部 + 全局建图节点：全局地图 4 Hz，局部地图 2.3 Hz
- 全局建图节点：7 - 10 Hz

如果发送 3 组语义则频率会更低一些。