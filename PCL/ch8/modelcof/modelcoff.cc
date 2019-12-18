#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

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

  std::cerr << "Cloud before projection: " << std::endl;
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;

  // 创建投影系数对象
  // X - Y 平面 ax + by + cz + d = 0，a = b = d = 0, c = 1
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = 0;
  coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;

  // 创建投影滤波器
  pcl::ProjectInliers<pcl::PointXYZ> proj;

  // 设置投影模型为平面
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);

  // 设置投影模型的系数为 X - Y 平面
  proj.setModelCoefficients (coefficients);

  // 执行滤波，保存结果
  proj.filter (*cloud_projected);

  std::cerr << "Cloud after projection: " << std::endl;
  for (std::size_t i = 0; i < cloud_projected->points.size (); ++i)
    std::cerr << "    " << cloud_projected->points[i].x << " " 
                        << cloud_projected->points[i].y << " " 
                        << cloud_projected->points[i].z << std::endl;

  return (0);
}
