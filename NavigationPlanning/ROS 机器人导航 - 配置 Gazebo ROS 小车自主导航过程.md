# ROS 配置 Gazabo 自主导航

## 安装导航包

### 1. Apt 安装：

这种适合不需要修改源码的同学：

```shell
sudo apt install ros-kinetic-navigation
```

### 2. 源码编译安装

推荐这种，因为可能自己要修改算法，从这个 [navigation-kinetic-devel](https://github.com/ros-planning/navigation/tree/kinetic-devel) 分支下载 zip 代码到本地，然后先使用下面的方法安装依赖：

```shell
# 编译过程可能会遇到找不到Bullet等依赖问题，解决这些依赖简便办法
# 先 apt-get 安装 ros-navigation 包，这样依赖会被自动装上 
sudo apt-get install ros-kinetic-navigation* 
# 再 apt-get 卸载掉 ros-navigation 包 
sudo apt-get remove ros-kinetic-navigation ros-kinetic-navigation-experimental
```

之所以不直接 git clone 是因为如果对 git 不熟悉的话，clone 下来的默认还是最新的版本，还需要 checkout 一次，所以直接下载 zip 比较妥当，重新 catkin_make 即可。我编译的时候第一次出现错误，然后我重新下载编译又可以了，暂时不知道为何。

### 3. 安装局部规划器

kinetic 默认的局部规划器比较老，可以使用这个比较新的，编译安装后即可使用：

```shell
https://github.com/rst-tu-dortmund/teb_local_planner.git
```

依然是选择 kinetic-devel 下载 zip 包，拷贝到工作空间先安装依赖：

```shell
rosdep install teb_local_planner
```

之后 catkin_make 不会出错。

## 配置自主导航

### 基本步骤

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

### 新建自己的导航包 agilex_navigation

agilex_navigation 用来存储导航用的配置文件和脚本。









### [map_server](http://wiki.ros.org/map_server)

读取地图参数，发布加载的地图主题 `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html))



## 遇到的问题

- 一边 SLAM 一边导航还需要使用 AMCL 吗？不需要，一边定位一边导航只需要使用 SLAM 和 move_base 即可
- `/odom` 从哪里订阅？
- `/odom` 的 tf 和 SLAM 定位的 tf 有和不同？









