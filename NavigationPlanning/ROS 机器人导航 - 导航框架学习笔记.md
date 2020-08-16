## ROS 自主导航

### 安装导航包

```shell
sudo apt install ros-kinetic-navigation
```

### 配置自主导航的步骤

1. 确定小车节点发布的话题
2. 配置 move_base 功能包，是否使用已经构建的地图？
3. 配置 AMCL 功能包？
4. 发布并订阅点云话题 sensor_msgs/PointCloud2
5. 发布并订阅 tf、Odom

注意 move_base 订阅的话题名是固定的：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/move_base.png)

### [map_server](http://wiki.ros.org/map_server)

读取地图参数，发布加载的地图主题 `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))





## 问题

- 一边 SLAM 一边导航还需要使用 AMCL 吗？不需要，一边定位一边导航只需要使用 SLAM 和 move_base 即可
- `/odom` 从哪里订阅？









