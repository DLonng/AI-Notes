为了在每帧点云中滤除噪声点，选择了半径滤波器，也用高斯滤波器测试过，但是没有半径效果好，这里记录下在 octomap_server 中增加半径滤波器的步骤，并在 launch 中配置滤波器参数。

### 1. 定义半径滤波器参数

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

### 2. 执行半径滤波

在 InsertPointCloudCallBack 函数的 PassThough 前执行半径滤波：

```cpp
// 对一帧 pc 点云进行半径滤波
pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;

// 这里需要传递指针，因为我的 pc 不是指针，所以这里做了 makeShared
outrem.setInputCloud(pc.makeShared());

outrem.setRadiusSearch(m_outrem_radius); 

outrem.setMinNeighborsInRadius (m_outrem_neighbors);

outrem.filter(pc);
```

### 3. 配置 launch 文件

```xml
<param name = "outrem_radius" type = "double" value = "1.0">
<param name = "outrem_neighbors" type = "int" value = "10">
```

这样以后就可以从 launch 中直接配置滤波器的参数了，不用每次修改再重新编译，这样调试起来非常方便。