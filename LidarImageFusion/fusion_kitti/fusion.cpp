#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>



int main() {

  // 读取点云
  std::stringstream ss;
  ss.str("");
  ss << "../data/raw_cloud.pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ss.str(), *raw_cloud) == -1) {
    std::cout << "Raw Cloud reading failed." << std::endl;
    return -1;
  } else {
    std::cout << "Raw Cloud reading success." << std::endl;
  }

  // 读入图像
  ss.str("");
  ss << "../data/raw_img.png";
  cv::Mat raw_img = cv::imread(ss.str(), CV_LOAD_IMAGE_COLOR);
  unsigned int img_width = raw_img.cols;
  unsigned int img_height = raw_img.rows;

  if (raw_img.data == nullptr) {
    std::cout << "Raw Img reading failed." << std::endl;
    return -1;
  } else {
    std::cout << "Raw Img reading success." << std::endl;
    cv::imshow("Raw Img", raw_img);
    //cv::waitKey(0);
  }

  //
  cv::Mat_<cv::Vec3b> raw_img_vec = raw_img;

  // 设置内参数矩阵 2011_09_26_drive_0005_sync
  // K_03: 9.037596e+02 0.000000e+00 6.957519e+02 0.000000e+00 9.019653e+02 2.242509e+02 0.000000e+00 0.000000e+00 1.000000e+00
  float fx = 9.037596e+02;
  float fy = 9.019653e+02;
  float cx = 6.957519e+02;
  float cy = 2.242509e+02;

  // 设置雷达到相机的 4X4 RT 矩阵 2011_09_26_drive_0005_sync
  // R: 7.533745e-03 -9.999714e-01 -6.166020e-04 1.480249e-02 7.280733e-04 -9.998902e-01 9.998621e-01 7.523790e-03 1.480755e-02
  // T: -4.069766e-03 -7.631618e-02 -2.717806e-01
  cv::Mat RT(4, 4, cv::DataType<double>::type);
  RT.at<double>(0, 0) = 7.533745e-03;
  RT.at<double>(0, 1) = -9.999714e-01;
  RT.at<double>(0, 2) = -6.166020e-04;
  RT.at<double>(0, 3) = -4.069766e-03;
  RT.at<double>(1, 0) = 1.480249e-02;
  RT.at<double>(1, 1) = 7.280733e-04;
  RT.at<double>(1, 2) = -9.998902e-01;
  RT.at<double>(1, 3) = -7.631618e-02;
  RT.at<double>(2, 0) = 9.998621e-01;
  RT.at<double>(2, 1) = 7.523790e-03;
  RT.at<double>(2, 2) = 1.480755e-02;
  RT.at<double>(2, 3) = -2.717806e-01;
  RT.at<double>(3, 0) = 0.0;
  RT.at<double>(3, 1) = 0.0;
  RT.at<double>(3, 2) = 0.0;
  RT.at<double>(3, 3) = 1.0;


  // 循环投影每一个点云
  std::vector<pcl::PointXYZ> cam_cloud(raw_cloud->points.size());
  cv::Mat raw_point(4, 1, cv::DataType<double>::type);
  cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

  pcl::PointXYZRGB colored_3d_point;

  // 经过 RT 矩阵转换后的点云坐标
  double x;
  double y;
  double z;

  // 存储融合后的彩色点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  out_cloud->points.clear();

  // 目前使用直接计算来进行投影融合
  // 还可以直接使用矩阵相乘来计算，参考 Uda code
  for (size_t i = 0; i < raw_cloud->points.size(); i++) {
    //cv::Vec3b raw_point(raw_cloud->points[i].x, raw_cloud->points[i].y, raw_cloud->points[i].z);
    //cv::Vec3b transformed_point = RT * raw_point;

    raw_point.at<double>(0, 0) = raw_cloud->points[i].x;
    raw_point.at<double>(1, 0) = raw_cloud->points[i].y;
    raw_point.at<double>(2, 0) = raw_cloud->points[i].z;
    raw_point.at<double>(3, 0) = 1;

    // 4 X 1 = 4 X 4 * 4 X 1;
    transformed_point = RT * raw_point;

    x = transformed_point.at<double>(0, 0);
    y = transformed_point.at<double>(1, 0);
    z = transformed_point.at<double>(2, 0);

    // 使用相机内参将三维空间点投影到像素平面
    int col = int(x * fx / z + cx);
    int row = int(y * fy / z + cy);

    // 只融合图像区域对应的点云
    if ((col >= 0) && (col < img_width) && (row >= 0) && (row < img_height) && (z > 0)) {

      colored_3d_point.x = raw_cloud->points[i].x;
      colored_3d_point.y = raw_cloud->points[i].y;
      colored_3d_point.z = raw_cloud->points[i].z;

      cv::Vec3b rgb_pixel = raw_img.at<cv::Vec3b>(row, col);
      colored_3d_point.r = rgb_pixel[2];
      colored_3d_point.g = rgb_pixel[1];
      colored_3d_point.b = rgb_pixel[0];

      out_cloud->points.push_back(colored_3d_point);
    }
  }

  pcl::visualization::PCLVisualizer viewer("Cloud viewer");
  viewer.addPointCloud(out_cloud, "Fusion cloud");
  viewer.setBackgroundColor(0, 0, 0);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }

  // 不能保存融合后的点云 Bug
  //pcl::io::savePCDFile("fusion.pcd", *out_cloud);
  //std::cerr << "Saved " << out_cloud->points.size() << "data points to fusion.pcd" << std::endl;


  return 0;
}
