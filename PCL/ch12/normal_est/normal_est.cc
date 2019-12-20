#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int, char** argv) {
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) { // load the file
    PCL_ERROR ("Couldn't read file");
    return -1;
  }

  std::cout << "points: " << cloud->points.size () << std::endl;

  // 创建法线估计对象
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  
  // 输入点云
  normal_estimation.setInputCloud (cloud);

  // 创建空 kdtree 对象，基于给定的输入点云数据建立 kdtree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  
  // 设置法线估计对象的查询方法为 kdtree
  normal_estimation.setSearchMethod (tree);

  // 储存输出的点云法线
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // 对于每个点都用半径为 3cm 的近邻搜索方式
  normal_estimation.setRadiusSearch (0.03);

  // 搜索方式也可以设置为 K 近邻搜索，比如取 K = 10
  // normal_estimation.setKSearch(10);

  // 计算法线
  normal_estimation.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
  std::cout << "cloud_normals->points.size (): " << cloud_normals->points.size () << std::endl;

  // visualize normals
  // add to pcl_github
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }

  return 0;
}
