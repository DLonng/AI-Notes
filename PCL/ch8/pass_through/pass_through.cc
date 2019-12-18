#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // 创建直通滤波器对象
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  
  // 设置过滤字段为 z 轴坐标
  pass.setFilterFieldName ("z");
  
  // 设置过滤范围为 (0.0, 1.0)
  pass.setFilterLimits (0.0, 1.0);
  
  // 过滤掉不符合条件的点云，打开注释过滤掉符合条件的点云
  pass.setFilterLimitsNegative (true); 
  
  // 过滤结果保存在 cloud_filtered 中
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;

  return (0);
}

