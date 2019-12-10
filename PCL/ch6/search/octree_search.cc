#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>


int main() {
  srand ((unsigned int)time(nullptr));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }

  float resolution = 128.0f;

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

  octree.setInputCloud(cloud);

  octree.addPointsFromInputCloud();

  pcl::PointXYZ search_point;

  search_point.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  search_point.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  search_point.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  std::vector<int> point_idx_vec;
  if (octree.voxelSearch(search_point, point_idx_vec)) {

    std::cout << "Neighbors within voxel search at (" << search_point.x
              << " " << search_point.y
              << " " << search_point.z << ")"
              << std::endl;

    for (std::size_t i = 0; i < point_idx_vec.size (); ++i) {
      std::cout << "    " << cloud->points[point_idx_vec[i]].x
                << " " << cloud->points[point_idx_vec[i]].y
                << " " << cloud->points[point_idx_vec[i]].z << std::endl;
    }

  }

  int k = 10;

  std::vector<int> point_idx_nkn_search;
  std::vector<float> point_nkn_square_distance;

  std::cout << "K nearest neighbor search at (" << search_point.x
            << " " << search_point.y
            << " " << search_point.z
            << ") with K=" << k << std::endl;

  if (octree.nearestKSearch(search_point, k, point_idx_nkn_search, point_nkn_square_distance) > 0) {
    for (std::size_t i = 0; i < point_idx_nkn_search.size (); ++i) {
      std::cout << "    "  <<   cloud->points[ point_idx_nkn_search[i] ].x
                << " " << cloud->points[ point_idx_nkn_search[i] ].y
                << " " << cloud->points[ point_idx_nkn_search[i] ].z
                << " (squared distance: " << point_nkn_square_distance[i] << ")" << std::endl;
    }
  }


  std::vector<int> point_idx_rad_search;
  std::vector<float> point_rad_square_distance;

  float rad = 256.0f * rand() / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << search_point.x
            << " " << search_point.y
            << " " << search_point.z
            << ") with radius=" << rad << std::endl;


  if (octree.radiusSearch(search_point, rad, point_idx_rad_search, point_rad_square_distance) > 0) {
     for (std::size_t i = 0; i < point_idx_rad_search.size (); ++i) {
      std::cout << "    "  <<   cloud->points[ point_idx_rad_search[i] ].x
                << " " << cloud->points[ point_idx_rad_search[i] ].y
                << " " << cloud->points[ point_idx_rad_search[i] ].z
                << " (squared distance: " << point_rad_square_distance[i] << ")" << std::endl;
    }
   
  }






  return 0;
}
