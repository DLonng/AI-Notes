#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>


int main() {
  // import pcd 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("./test_data/color_cloud.pcd", *color_cloud)) {
    std::cout << "color_cloud.pcd reading failed." << std::endl;
    return -1;
  } else {
    std::cout << "color_cloud.pcd reading success." << std::endl;
  }
  
  // read img
  cv::Mat raw_img = cv::imread("./test_data/raw_img.png", CV_LOAD_IMAGE_COLOR);
  if (raw_img.data == nullptr) {
    std::cout << "raw_img.png reading failed." << std::endl;
    return -1;
  } else {
    std::cout << "raw_img.png reading success." << std::endl;
  }

  // load calib.txt using cv::Mat
 
  
  // 对点云进行预处理，理解点云结构, 考虑是否要分割出地面？
  

  // 计算空间距离，每个线点云中的 2 个点云距离都相同？
  
  
  // 计算插值点数量
  
  
  // 计算插值点空间坐标 X, Y, Z
  
  
  // 根据 img 确定插值点的 RGB 值
  
  
  // 显示插值后的点云文件
  
  
  // 保存为 color_cloud_inter.pcd





  return 0;
}
