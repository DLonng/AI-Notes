### 1. 增加 source 到 bashrc 中

为了不用在小车的 Ubuntu 上每次都 `source` 自己的工作空间，我把自己的 workspace 加到 `~/.bashrc` 文件末尾：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/zshrc_source.png)

然后重启 shell，之后就会自动 source 我得工作空间，可以使用 roscd 直接进入指定包中：

```shell
roscd pkg_name
```

### 2. 编写启动 launch

每次调试小车，都要分别启动很多节点，比如融合、建图、Loam、Rviz 等，所以我把这些节点全部配置在一个 launch 中启动，直接一行命令启动，这样调试就方便多了，大大节省时间：

- `lidar_camera_fusion\launch`：`lidar_camera_fusion.launch`
- `octomap_server\rviz`：`octomap_debug.rviz`
- `octomap_server\launch`：`dlonng_octomap_test.launch`

```xml
<launch>
  <!-- Start ZED! -->
  <include file = "$(find zed_cpu_ros)/launch/zed_cpu_ros.launch" />

  <!-- Start Robosense LIDAR! -->
  <include file = "$(find rslidar_pointcloud)/launch/rs_lidar_16.launch" />

  <!-- Start fusion node! -->
  <include file = "$(find lidar_camera_fusion)/launch/lidar_camera_fusion.launch" />

  <!-- Start Octomap node! -->
  <include file = "$(find octomap_server)/launch/octomap_server_start.launch" />

  <!-- Start octomap_debug.rviz -->
  <node pkg = "rviz" type = "rviz" name = "$(anon rviz)" respawn = "false" output = "screen" args = "-d $(find octomap_server)/rviz/octomap_debug.rviz"/>

  <!-- Start Lego Loam! -->
  <include file = "$(find lego_loam)/launch/demo.launch" />

  <!-- Only using in map build test! -->
  <node pkg = "tf2_ros" type = "static_transform_publisher" name = "dlonng_static_test_broadcaster" args = "0 0 0 0 0 0 base_link rslidar" />

</launch>
```

### 3. 补充一些小 Tips

如果不知道要启动的包的 launch 文件叫啥，可以查找指定包的路径，然后进入看看 launch 文件夹下面的启动文件叫啥：

```shell
rospack list | grep "zed_cpu_ros"
```

```shell
rospack list | grep "rslidar_pointcloud"
```

之后我就可以从 octomap_server 中一键启动所有我建图要用的节点了：

```shell
roslaunch octomap_server dlonng_octomap_test.launch
```

但是启动过程中遇到 rviz 节点名字重复的问题，我把雷达启动的 RVIZ 关掉或者把启动 RVIZ 的节点名称改变下，防止重复。