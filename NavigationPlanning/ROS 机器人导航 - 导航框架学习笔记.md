## ROS 自主导航

### 安装导航包

Apt 安装：

```shell
sudo apt install ros-kinetic-navigation
```

源码编译安装，推荐这种，因为可能自己要修改算法：https://github.com/ros-planning/navigation/tree/kinetic-devel

```shell
# 编译过程可能会遇到找不到Bullet等依赖问题，解决这些依赖简便办法
# 先 apt-get 安装 ros-navigation 包，这样依赖会被自动装上 
sudo apt-get install ros-kinetic-navigation* 
# 再 apt-get 卸载掉 ros-navigation 包 
sudo apt-get remove ros-kinetic-navigation ros-kinetic-navigation-experimental
```

最后重新 catkin_make 导航包。

### 配置自主导航的步骤

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

### [map_server](http://wiki.ros.org/map_server)

读取地图参数，发布加载的地图主题 `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))



## 问题

- 一边 SLAM 一边导航还需要使用 AMCL 吗？不需要，一边定位一边导航只需要使用 SLAM 和 move_base 即可
- `/odom` 从哪里订阅？
- `/odom` 的 tf 和 SLAM 定位的 tf 有和不同？









