## 一、Rviz 可视化 Octomap

### 1.1 可视化插件

可视化需要使用 octomap rviz 插件，如果没有安装，使用下面的命令，kinetic 替换为你的 ROS 版本：

```shell
sudo apt-get install ros-kinetic-octomap-rviz-plugins
```

该插件使得 RVIZ 能够订阅八叉树的主题，其中的 OccupancyGrid 是显示三维概率地图，也就是 octomap 地图，而 OccupancyMap 是显示二维占据栅格地图：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rviz_octomap.png)

### 1.2 RVIZ 可视化地图步骤

RVIZ 可视化八叉树地图思路，以下主题名需要根据实际更改：

1. 将雷达点云发布到 ROS 主题（topic）中，比如 `/points_cloud`
2. 启动 octomap 节点订阅第一步发布的点云主题 `/points_cloud`，然后发布当前节点创建八叉树地图 topic，比如 `/octomap_show`
3. 启动 RVIZ 并订阅 `/octomap_show` 主题，即可实时显示

### 1.3 可视化使用的相关命令

查看所有发布的主题：

```shell
rostopic list

/points_cloud
```

查看单个主题发布的消息：

```shell
rostopic echo /points_cloud
```

启动 RVIZ：

```shell
rosrun rviz rviz
```

RVIZ 订阅主题的基本步骤、以添加 `/PointCloud2` 主题为例：

1. 点击 `Add` 添加要订阅的主题类型

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rviz_add.png)

2. 选择添加订阅 `/PointCloud2` 主题，然后设置 `Fixed Frame` 为订阅主题的 `header.frame_id`，设置 `Topic` 为要订阅的主题名，比如 `/pointcloud/output`

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rviz_frame_topic.png)

## 二、可视化详细步骤和代码

### 2.1 编写点云主题发布节点

```cpp
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

int main(int argc, char** argv)
{
    std::string topic, path, frame_id;
    int hz = 5;

    ros::init(argc, argv, "publish_pointcloud");
    ros::NodeHandle nh;

    nh.param<std::string>("path", path, "/修改路径/test.pcd");
    nh.param<std::string>("frame_id", frame_id, "camera");
    nh.param<std::string>("topic", topic, "/pointcloud/output");
    nh.param<int>("hz", hz, 5);
  
    // 主题发布者
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(topic, 10);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile(path, cloud);
  
    // 转换成ROS下的数据类型 最终通过topic发布
    pcl::toROSMsg(cloud, output); 

    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id;

    cout << "path = " << path << endl;
    cout << "frame_id = " << frame_id << endl;
    cout << "topic = " << topic << endl;
    cout << "hz = " << hz << endl;

    ros::Rate loop_rate(hz);
    while (ros::ok()) {
        // 发布主题
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
```

### 2.2 编写 octomap_server 启动文件

编写 `octomap_server_start.launch` 文件：

```xml
<launch>
  <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">

    <!-- resolution in meters per pixel -->
    <param name="resolution" value="0.05" />

    <!-- name of the fixed frame, needs to be "/map" for SLAM -->
    <!-- 
         发布静态全局地图帧的 header.frame_id = camera 这里指定的就是相机（因为这个例子的点云是通过 RGBD 相机产生的）
         也可以指定其他静态全局地图的 frame_id，但在增量式构建地图时，需要提供输入的点云帧和静态全局帧之间的 TF 变换
    -->
    <param name="frame_id" type="string" value="camera" />

    <!-- max range / depth resolution of the kinect in meter -->
    <param name="sensor_model/max_range" value="100.0" />
    <param name="latch" value="true" />

    <!-- max/min height for occupancy map, should be in meters -->
    <param name="pointcloud_max_z" value="1000" />
    <param name="pointcloud_min_z" value="0" />

    <!-- topic from where pointcloud2 messages are subscribed -->
    <!-- 要订阅的点云主题名称 /pointcloud/output -->
    <!-- 这句话的意思是把当前节点订阅的主题名称从 cloud_in 变为 /pointcloud/output -->
    <remap from="/cloud_in" to="/pointcloud/output" />
 
  </node>
</launch>
```

### 2.3 测试

这里就不讲如何编译节点了，直接启动点云发布节点：

```shell
rosrun publish_pointcloud publish_pointcloud
```

再启动 octomap_server 节点：

```shell
roslaunch publish_pointcloud octomaptransform.launch
```

该节点启动后，会有一个 `octomap_full` 主题，这个主题发布的就是生成的八叉树网格地图，使用 `rostopic` 查看：

```shell
rostopic list

...
/octomap_full
...
```

在 RVIZ 中订阅一个 `OccupancyGrid` 类型的主题：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/rviz_octomap.png)

设置主题的 Topic 为 `/octomap_full` 即可实时显示经过 `octomap_server` 节点生成的八叉树占用网格地图，退出后可以保存当前 RVIZ 的配置为 `octomap_show.rviz`，方便后面配置一键启动：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/octomap_server_rviz.png)

还可以调整体素网格大小和颜色等，自己尝试下吧。

### 2.4 一键启动 

以上是分开启动 2 个节点和一个 RVIZ，也可以在一个 `gogogo.launch` 文件中同时启动：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- 点云发布节点 publish_pointcloud -->
  <node name="publish_pointcloud" pkg="publish_pointcloud" type="publish_pointcloud">
	<param name="path" value="$(find publish_pointcloud)/data/test.pcd" type="str" />
	<param name="frame_id" value="camera" type="str" />
	<param name="topic" value="/pointcloud/output" type="str" />
	<param name="hz" value="2" type="int" />
  </node>

  <!-- 寻找启动 octomap_server 节点的 launch 文件，该节点也在 publish_pointcloud 包中 -->
  <include file="$(find publish_pointcloud)/launch/octomap_server_start.launch" />

  <!-- RViz -->
  <node pkg="rviz" type="rviz" name="$(anon rviz)" respawn="false" output="screen" args="-d $(find publish_pointcloud)/rviz/octomap_show.rviz"/>

</launch>
```

注意文件夹结构如下：

- publish_pointcloud
  - launch
    - octomap_server_start.launch
    - gogogo.launch
  - rviz
    - octomap_show.rviz（上一步查看 RVIZ 保存的 RVIZ 配置文件）

### 注意

发布的主题名 topic 和 `frame_id` 要特别注意，没有数据输出通常都是因为这 2 者没有填对。

## 三、项目相关

本项目可视化八叉树只需要配置：

1. 编写语义融合节点，发布融合的点云主题 `/fusion_topic`
2. 启动 octomap_server 节点，订阅 `/fusion_topic`
3. 启动 RVIZ 订阅 octomap_server 发布的 `/octomap_full` 主题

相关问题：

1. 是否能在 RVIZ 中可视化融合后的八叉树的颜色呢？[评论回答可以显示颜色](https://blog.csdn.net/crp997576280/article/details/74605766)
2. 这个 octomap_server 节点是不是只能可视化单帧点云，不能建图？可以自动增量式建图！

## 参考博客

- [Octomap 在ROS环境下实时显示](https://blog.csdn.net/crp997576280/article/details/74605766)