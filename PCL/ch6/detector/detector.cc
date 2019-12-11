#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>


int main() {
  srand((unsigned int)time(nullptr));

  float resolution = 32.0f;

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA(new pcl::PointCloud<pcl::PointXYZ>);


  cloudA->width = 128;
  cloudA->height = 1;
  cloudA->points.resize (cloudA->width * cloudA->height);

  for (std::size_t i = 0; i < cloudA->points.size (); ++i) {
    cloudA->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudA->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudA->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  }

  octree.setInputCloud(cloudA);
  octree.addPointsFromInputCloud();

  octree.switchBuffers();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB(new pcl::PointCloud<pcl::PointXYZ>);

  cloudB->width = 128;
  cloudB->height = 1;
  cloudB->points.resize (cloudB->width * cloudB->height);

  for (std::size_t i = 0; i < cloudB->points.size (); ++i) {
    cloudB->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudB->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    cloudB->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  }

  octree.setInputCloud(cloudB);
  octree.addPointsFromInputCloud();

  std::vector<int> new_point_idx_vec;

  octree.getPointIndicesFromNewVoxels(new_point_idx_vec);

  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (std::size_t i = 0; i < new_point_idx_vec.size (); ++i) {
    std::cout << i << "# Index:" << new_point_idx_vec[i]
              << "  Point:" << cloudB->points[new_point_idx_vec[i]].x << " "
              << cloudB->points[new_point_idx_vec[i]].y << " "
              << cloudB->points[new_point_idx_vec[i]].z << std::endl;
  }



  return 0;
}
