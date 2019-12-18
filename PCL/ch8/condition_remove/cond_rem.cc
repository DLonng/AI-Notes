#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int main (int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  if (strcmp(argv[1], "-r") == 0) {
    
    // 创建（半径近邻）滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    
    // 设置输入点云
    outrem.setInputCloud(cloud);

    // 设置在 0.8 半径的范围内寻找近邻
    outrem.setRadiusSearch(0.8);

    // 滤除近邻数小于 1 个的数据点
    // 即必须有 1 个以上近邻的数据点才会保留
    outrem.setMinNeighborsInRadius (1);

    // 执行滤波
    outrem.filter (*cloud_filtered);

  } else if (strcmp(argv[1], "-c") == 0) {
    
    // 创建条件限定对象
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());

    // 为条件限定对象在 Z 轴上添加大于(Great Than) 0 的滤除条件
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                               pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.0)));
    
    // 为条件限定对象在 Z 轴上添加小于(Less Than) 0.8 的滤除条件
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
                               pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
    // 创建条件限定滤波器对象
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    
    // 设置滤波条件:点云 Z 必须在 (0, 0.8) 范围内，否则被滤出
    condrem.setCondition (range_cond);
    
    // 设置输入点云
    condrem.setInputCloud (cloud);
    
    // 设置保持原有点云结构，设置为 false 则不会将被滤出的点设置为 Nan
    condrem.setKeepOrganized(true);

    // 执行滤波
    condrem.filter (*cloud_filtered);
  
  } else {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
              << cloud->points[i].y << " "
              << cloud->points[i].z << std::endl;
  // display pointcloud after filtering
  std::cerr << "Cloud after filtering: " << std::endl;
  for (std::size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " "
              << cloud_filtered->points[i].y << " "
              << cloud_filtered->points[i].z << std::endl;
  return (0);
}
