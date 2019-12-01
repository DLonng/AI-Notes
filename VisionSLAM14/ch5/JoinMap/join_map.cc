#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <boost/format.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int main() {

  std::vector<cv::Mat> color_imgs;
  std::vector<cv::Mat> depth_imgs;

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

  ifstream fin("./pose.txt");
  if (!fin) {
    std::cerr << "no post.txt!" << std::endl;
    return 1;
  }

  for (int i = 0; i < 5; i++) {
    boost::format fmt("./%s/%d.%s");
    color_imgs.push_back(cv::imread((fmt%"color"%(i + 1)%"png").str()));
    depth_imgs.push_back(cv::imread((fmt%"depth"%(i + 1)%"pgm").str(), -1));

    double data[7] = { 0 };
    for (auto& d : data) {
      fin >> d;
    }

    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
    Eigen::Isometry3d T(q);
    T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
    poses.push_back(T);
  }

  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double depth_scale = 1000.0;

  std::cout << "image -> point cloud... ... ..." << std::endl;

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;

  PointCloud::Ptr point_cloud(new PointCloud);
  
  for (int i = 0; i < 5; i++) {
    std::cout << "convert img......" << std::endl;
    
    cv::Mat color = color_imgs[i];
    cv::Mat depth = depth_imgs[i];
    Eigen::Isometry3d T = poses[i];

    for (int v = 0; v < color.rows; v++) {
      for (int u = 0; u < color.cols; u++) {
        unsigned int d = depth.ptr<unsigned short>(v)[u];
        if (d == 0) {
          continue;
        }
        
        Eigen::Vector3d point;
        point[2] = double(d) / depth_scale;
        point[0] = (u - cx) * point[2] / fx;
        point[1] = (v - cy) * point[2] / fy;
        Eigen::Vector3d point_world = T * point;

        PointT p;
        p.x = point_world[0];
        p.y = point_world[1];
        p.z = point_world[2];
        p.b = color.data[v * color.step + u * color.channels()];
        p.g = color.data[v * color.step + u * color.channels() + 1];
        p.r = color.data[v * color.step + u * color.channels() + 2];
        
        point_cloud->points.push_back(p);
      }
    }
    
  }

  point_cloud->is_dense = false;
  std::cout << "point sum = " << point_cloud->size() << std::endl;
  pcl::io::savePCDFileBinary("map2.pcd", *point_cloud);

  return 0;
}
