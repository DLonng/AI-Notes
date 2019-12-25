#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


int main(int argc, char** argv) {
  
  // 新建点云存储对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

  // 读取文件
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0) {
    return -1;
  }
  
  // 滤波对象
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
  
  filter.setInputCloud(cloud);
  
  //建立搜索对象
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  
  filter.setSearchMethod(kdtree);
  
  //设置搜索邻域的半径为3cm
  filter.setSearchRadius(0.03);
  
  // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
  filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);

  filter.setPolynomialOrder(2);

  // 采样的半径是
  filter.setUpsamplingRadius(0.025);
  
  // 采样步数的大小
  filter.setUpsamplingStepSize(0.02);

  filter.process(*filteredCloud);

  pcl::io::savePCDFile("test_mls.pcd", *filteredCloud);

  return 0;
}
