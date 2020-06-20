# 在小车上测试融合与建图过程！

## 一、编译融合与建图节点

### 1.1 创建工作空间 dlonng_ws

```shell
# 创建工作空间
mkdir dlonng_ws
cd dlonng_ws/

# ROS 的工作空间必须包含 src 目录
mkdir src/

# 创建
catkin_make

# source 环境变量
source devel/setup.bash
```

### 1.2 编译 lidar_camera_fusion

把 lidar_camera_fusion 的代码拷贝到 src 目录下，然后开始编译。

#### 1.2.1 配置使用 ROS Kinetic OpenCV

我的台式机安装了 2 个版本的 OpenCV：

- 编译安装：opencv-3.4.3
- ROS Kinetic 自带：opencv-3.3.1-dev

为了方便，我在小车上的 Intel NUC 中直接使用 Kinetic 自带的 opencv：

```shell
ls /opt/ros/kinetic/include/opencv-opencv-3.3.1-dev
```

`CMakeLists.txt` 中 opencv-3.3.1 的配置如下：

```cmake
find_package(catkin REQUIRED COMPONENTS 
  OpenCV
)

# ROS opencv-3.3.1-dev 头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 链接 opencv-3.3.1-dev 库
target_link_libraries(your_exe_name ${OpenCV_LIBRARIES})
```

因为 Intel NUC 是新机器，很多环境都没配置，所以编译期间出了一个小问题，不能找到 opencv 的 cmake 配置：

```shell
 Could not find a package configuration file provided by "OpenCV" with any
  of the following names:

    OpenCVConfig.cmake
    opencv-config.cmake
```

我按照报错的提示，找了下解决方法，发现在 CMakeLists.txt 前面加上如下一行即可，这里的路径你可以在你的系统中 ls 一下，看看存不存在：

```cmake
...

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)

...

find_package(catkin REQUIRED COMPONENTS 
  OpenCV
)
```

然后在头文件中直接导入 3.3.1，注意前缀：

```cpp
#include <opencv-3.3.1-dev/opencv2/core.hpp>
#include <opencv-3.3.1-dev/opencv/highgui.h>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
```

#### 1.2.2 另一种方式： 自己编译安装 opencv - 3.4.3

如果想用自己编译的 opencv 也没问题，就是改下 CMakeLists.txt 即可：

```cmake
find_package(OpenCV REQUIRED)

# 查找 OpenCV-3.4.3 头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# 链接 OpenCV 库
target_link_libraries(your_exe_name ${OpenCV_LIBS})
```

然后 include 的头文件名字如下：

```cpp
#include <opencv2/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
```

#### 1.2.3 编译

```shell
catkin_make
```

#### 1.2.4 启动测试

```shell
roslaunch lidar_camera_fusion lidar_camera_fusion.launch
```

参考资料：

- [ROS Kinetic 中自带的 OpenCV 使用测试](https://blog.csdn.net/yuguo0_331/article/details/82656568?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-1.nonecase)
- [ROS Kinetic 中 OpenCV 使用](https://blog.csdn.net/u010284636/article/details/80071786?utm_medium=distribute.pc_relevant.none-task-blog-baidujs-3)

### 1.3 编译 octomap_mapping

再拷贝八叉树建图包到 src 目录下，然后编译：

```shell
rosdep install octomap_mapping

catkin_make
```

但是出现了一个跟编译 OpenCV 差不多的问题，找不到 octomap_ros 的 cmake 配置：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/octomap_build_problem_nuc.jpg)

我回想了下我在台式机上的编译过程，没有遇到这个问题，可能是因为我在台式机上先用 apt 安装了 octomap，于是我直接 apt 安装以下的 octomap 组件，除了 octomap_server 和 octomap_mapping，因为这两个我是要自己编译：

```shell
sudo apt-get install ros-kinetic-octomap  ros-kinetic-octomap-msgs ros-kinetic-octomap-ros ros-kinetic-octomap-pa ros-kinetic-octomap-rviz-plugins 
```

安装完后，我**重启终端**，然后再编译就通过了，也能正常启动：

```shell
roslaunch octomap_server octomap_mapping.launch
```

## 二、启动过程

### 2.1 启动 ROS

```shell
roscore
```

### 2.2 启动 Robosense 雷达

```shell
roslaunch rslidar_pointcloud rs_lidar_16.launch
```

### 2.3 启动 ZED 相机

因为我们的 Intel NUC 没有英伟达显卡，所以官方的驱动用不了， 在 Github 找了一个 zed_cpu_ros 相机驱动，但是有个 bug 需要先开启下 ZED_Explorer，才能正确启动相机：

```shell
cd /usr/local/zed/tools/

./ZED_Explorer
```

启动 ZED：

```shell
roslaunch zed_cpu_ros zed_cpu_ros.launch
```

### 2.4 启动融合节点

```shell
roslaunch lidar_camera_fusion lidar_camera_fusion.launch
```

### 2.5 启动 Rviz

```shell
rosrun rviz rviz
```

查看融合主题是否正常输出，这里经常不能显示出 Image，需要自己先 Remove Image 然后重新 Add，这个问题暂时还没解决，但是不影响。

### 2.6 启动建图节点

```shell
roslaunch octomap_server octomap_mapping.launch
```

修改 `frame_id` 为 `camera_init`，只用于测试！

### 2.7 启动 Loam

用师兄的 Loam 获取定位：

```shell
roslaunch lego_loam demo.launch
```

这里会重开一个 Rviz。

### 2.8 查看系统  TF Tree

```shell
rosrun rqt_tf_tree rqt_tf_tree
```

建图节点使用的是 「world -> rslidar」 的 TF，为了测试，我直接发布一个 「base_link -> rslidar」 的静态变换：

```shell
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar
```

然后把 octomap 的 launch 文件中的 world frame 设置为 camera_init：

```xml
<param name="frame_id" type="string" value="camera_init" />
```

这样系统中就存在「camera_init -> rslidar」的 TF 了：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/mini_test_tf_tree.png)

### 2.9 查看 ROS_DEBUG

如何查看 ROS_DEBUG 的日志？目前只能显示 INFO

## 三、建图过程

### 3.1 调整分辨率

分别调整体素栅格的边长（m）：

- 0.01
- 0.05
- 0.1
- ...

边长越小，分辨率越高，但是计算越慢！0.01 会卡！

### 3.2 保存八叉树地图

编译后的 octomap_saver 在 devel/lib/octomap_server 中：

```shell
ls  devel/lib/octomap_server
```

拷贝到 /usr/bin 下，方便以后使用：

```shell
sudo cp devel/lib/octomap_server/octomap_saver /usr/bin/
```

保存完整的概率地图：

```shell
octomap_saver -f mapfile.ot
```

保存压缩的二进制格式的地图：

```shell
octomap_saver mapfile.bt
```

### 3.3 显示八叉树地图

安装八叉树可视化程序 octovis，安装后重启终端：

```shell
sudo apt-get install octovis
```

显示八叉树地图：

```shell
octovis xxx.ot[bt]
```

保存的八叉树地图不能显示颜色信息！需要解决！小车跑的可以保存带颜色的地图。

## 四、调整建图参数

### 4.1 测试 latch 

 latch 设置为 false 能够加快建图速度，取消每次地图的改变都发布所有主题和可视化对象的功能：

```xml
<param name = "latch" value = "false">
```

### 4.2 移动物体

- 小车不动，视野内的物体运动，运动轨迹会自动清除！
- 小车运动，视野内的物体运动，观察当运动物体离开视野后，运动轨迹是否存在？存在一些！需要解决这个问题
- resolution：0.1，max_range 20m

## 五、航院数据集

### 5.1 测试目的

- 测试野外建图效果，查找问题
- 录制数据集

### 5.2 建图结果

- 分辨率：20cm，录制频率回放

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/HangYuan-2020-06-18-17-00-0.20m.png)

- 分辨率：15cm，录制频率回放

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/HangYuan-2020-06-18-17-00-0.15m.png)

- 分辨率：10cm，录制频率回放

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/HangYuan-2020-06-18-17-00-0.10m.png)

- 分辨率：10cm，0.5 倍速回放

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/HangYuan-2020-06-18-17-00-0.10m-0.5r.png)

- 分辨率：5cm，0.5 倍数回放

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/HangYuan-2020-06-18-17-00-0.05m-0.5r.png)

### 5.3 野外建图存在的问题

- 运动轨迹没有测试，还要多测试测试！
- 分辨率不高的情况，地面不会完全填充满，存在空点（同一路线，多跑几次，来回跑，可否这地图构建稠密？）
- 把右边相机也融合点云，加入地图中？
- 融合节点频率
- 建图节点频率

## 七、地图滤波

### 7.1 半径滤波器

原始地图分辨率 15cm：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/origin_hangyuan_15cm.png)

半径滤波结果，滤波半径 1m，滤除 1m 半径内少于 10 个邻居的点云（这个参数还可以）：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/outrem_result_1m_10Neighbors.png)

原始地图分辨率 10cm，0.5 r 倍速回放：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/origin_hangyuan_10cm_0.5r.png)

半径滤波结果，滤波半径 1m，滤除 1m 半径内少于 10 个邻居的点云：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/outrem_hangyuan_10cm_0.5r_1m_10n.png)

### 7.2 高斯滤波器

