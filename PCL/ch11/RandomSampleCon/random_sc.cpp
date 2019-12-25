#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>


// using namespace std::chrono_literals; // Fix bug: set( CMAKE_CXX_FLAGS "-std=c++11") -> set( CMAKE_CXX_FLAGS "-std=c++14")
using namespace std::literals::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv) {
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  // populate our PointCloud with points
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->points.size (); ++i) {
    // -s or -sf 创建立方体包围的球体模型
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0) {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                    - (cloud->points[i].y * cloud->points[i].y));
      else
        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
    } else {
      // 创建平面模型
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }

  std::vector<int> inliers;

  // 创建球面模型随机采样一致性对象
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_s(
      new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

  // 创建平面模型随机采样一致性对象
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

  if(pcl::console::find_argument (argc, argv, "-f") >= 0) {
    
    // 估计平面模型，存储计算的局内点
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);

    // 与平面距离小于 0.01 的点作为局内点
    ransac.setDistanceThreshold (.01);
    
    // 估计模型的参数
    ransac.computeModel();
    
    // 存储计算的内点
    ransac.getInliers(inliers);
  } else if (pcl::console::find_argument (argc, argv, "-sf") >= 0 ) {
    
    // 估计球面模型，存储计算的局内点
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    
    // 与球面距离小于 0.01 的点作为局内点
    ransac.setDistanceThreshold (.01);
    
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // copies all inliers of the model computed to another PointCloud
  pcl::copyPointCloud (*cloud, inliers, *final);

  // creates the visualization object and adds either our original cloud or all of the inliers
  // depending on the command line arguments specified.
  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    viewer = simpleVis(final);
  else
    viewer = simpleVis(cloud);

  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    std::this_thread::sleep_for(100ms);
  }
  
  return 0;
}


