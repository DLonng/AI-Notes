为了在每帧点云中滤除噪声点，选择了半径滤波器，也用高斯滤波器测试过，但是没有半径效果好，这里记录下在 octomap_server 中增加半径滤波器的步骤，并在 launch 中配置滤波器参数。

## 一、半径滤波器基本原理

放一张汇报用的 PPT 截图：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/banjing_filter.png)

原理很简单就是判断一个点云周围（半径 R）有没有足够多（K）的邻居点，如果没有就删除这个点，否则就保留。

## 二、基本用法

我一般学习技术喜欢到官网看最原始的教程：[Removing outliers using a Conditional or RadiusOutlier removal](https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html#remove-outliers)，这个教程介绍了半径滤波器（我不清楚中文名到底叫什么滤波器）的基本用法：

```cpp
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>

// 输入待滤波的原始点云指针
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

// 保存滤波后的点云指针
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

// 创建滤波器对象
pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

// 设置要滤波的点云
outrem.setInputCloud(cloud);

// 设置滤波半径
outrem.setRadiusSearch(0.8);

// 设置滤波最少近邻数
outrem.setMinNeighborsInRadius (2);

// 执行半径滤波
outrem.filter (*cloud_filtered);
```

如果第一次使用 PCL 的滤波器，可以把这个教程自己运行一遍，我之前运行过了，这次就不贴代码了，下面分享下我在实际项目中如果使用这个半径滤波器对我的 octomap_server 构建的八叉树地图进行滤波。

## 三、给我的地图滤波

### 3.1 定义半径滤波器参数

半径滤波器有 2 个参数：滤波半径和半径内部邻居数，注意数据类型

```cpp
// 滤波半径
double m_outrem_radius;

// 半径内的邻居数
int m_outrem_neighbors;
```

在构造函数初始化列表中初始化：

```cpp
OctomapServer::OctomapServer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: ...,
  m_outrem_radius(-std::numeric_limits<double>::max()),
  m_outrem_neighbors(-std::numeric_limits<int>::max()),
  ...
```

从 launch 中读取启动参数：

```cpp
// add outrem filter
m_nh_private.param("outrem_radius", m_outrem_radius, m_outrem_radius);
m_nh_private.param("outrem_neighbors", m_outrem_neighbors, m_outrem_neighbors);
```

### 3.2 执行半径滤波

在 InsertPointCloudCallBack 函数的 PassThough 前执行半径滤波，即对每一帧点云在构建八叉树地图前进行滤波，主要是为了去掉单独的离群点：

```cpp
// 对一帧 pc 点云进行半径滤波
pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;

// 这里需要传递指针，因为我的 pc 不是指针，所以这里做了 makeShared
outrem.setInputCloud(pc.makeShared());

// 设置滤波半径，这里设置为 1m
outrem.setRadiusSearch(m_outrem_radius); 

// 设置滤波近邻数，这里设置为 10 个
outrem.setMinNeighborsInRadius (m_outrem_neighbors);

// 执行滤波
outrem.filter(pc);
```

### 3.3 在 launch 中配置半径滤波器参数

```xml
<param name = "outrem_radius" type = "double" value = "1.0">
<param name = "outrem_neighbors" type = "int" value = "10">
```

这样以后就可以从 launch 中直接配置滤波器的参数了，不用每次修改再重新编译，这样调试起来非常方便。

### 3.4 滤波结果

这是原始地图，15cm 分辨率，红框内部有很多单个的点：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/banjing_filter_before.png)

这是滤波后的效果，滤波半径 1m，近邻点 10 个：

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/banjing_filter_after.png)

效果还是可以的，希望能对你有帮助，如果使用其他的滤波器，按照官方的教程来就行了，掌握学习方法才是最重要的：）