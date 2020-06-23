在图像和点云的融合节点中，做了一个点云格式的转换：

- ROS 点云 sensor_msgs::PointCloud2 -> PCL 第一代点云 pcl::PointCloud<pcl::PointXYZ>

这里记录下常用的 ROS 和 PCL 之间的转换。

### 1. sensor_msgs::PCLPointCloud2 <-> pcl::PointCloud<pcl::PointXYZ>

把 ROS PointCloud2 转为 PCL 第一代 PointCloud，方便用 PCL 库处理：

```cpp
void pcl::fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &);
```

比如：

```cpp
// ROS 点云
sensor_msgs::PointCloud2::ConstPtr& cloud_msg;

// PCL 第一代点云
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

// ROS 点云 -> PCL 第一代点云
pcl::fromROSMsg(*cloud_msg, *pcl_cloud_msg);
```

把 PCL 第一代 PointCloud 转为 ROS PointCloud2，用于发布 ROS 的点云主题：

```cpp
void pcl::toROSMsg(const pcl::PointCloud<T> &, sensor_msgs::PointCloud2 &);
```

比如：

```cpp
// PCL 第一代点云
pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

// ROS 点云
sensor_msgs::PointCloud2 fusion_cloud;

// PCL 第一代点云 -> ROS 点云
pcl::toROSMsg(*out_cloud, fusion_cloud);
```

### 2. sensor_msgs::PCLPointCloud2 <=> pcl::PCLPointCloud2

所用的头文件：

```cpp
#include <pcl_conversions/pcl_conversions.h>
```

把 ROS PointCloud2 转为 PCL 第二代 PointCloud2：

```cpp
void pcl_conversions::toPCL(const sensor_msgs::PointCloud2 &, pcl::PCLPointCloud2 &)
```

比如：

```cpp
// ROS 点云 -> 第二代 PCL 点云
// cloud_msg 和 pcl_cloud2 这里都定义为指针
pcl_conversions::toPCL(*cloud_msg, *pcl_cloud2);
```

把 PCL 第二代 PointCloud2 转为 ROS PointCloud2：

```cpp
void pcl_conversions::moveFromPCL(const pcl::PCLPointCloud2 &, const sensor_msgs::PointCloud2 &);
```

比如：

```cpp
pcl::PCLPointCloud2 pcl2_cloud_filtered;
sensor_msgs::PointCloud2 ros_cloud_filter;

// 第二代 PCL 点云 -> ROS 点云
pcl_conversions::fromPCL(pcl2_cloud_filtered, ros_cloud_filter);
```

参考链接：

- https://www.cnblogs.com/li-yao7758258/p/6651326.html