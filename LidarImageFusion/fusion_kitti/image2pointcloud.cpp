#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


std::stringstream ss;

std::vector<Eigen::Matrix4f> RT;
std::vector<Eigen::Matrix4f> INV;


void InitMatrix();

std::vector<cv::Point3d> Generate3DPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num);

int main() {
    Eigen::Matrix4f rt0, rt1, rt2, rt3, rt4;
    rt0 << 0, 0, -1, 0,  0, 1, 0, 0,  1, 0, 0, 0, 0, 0, 0, 1;
    rt1 << 0, 0, -1, 0,  -0.95105651629, 0.30901699437, 0, 0,  0.30901699437, 0.95105651629, 0, 0, 0, 0, 0, 1;
    rt2 << 0, 0, -1, 0,  -0.58778525229, -0.80901699437, 0, 0,  -0.80901699437, 0.58778525229, 0, 0, 0, 0, 0, 1;
    rt3 << 0, 0, -1, 0,  0.58778525229, -0.80901699437, 0, 0,  -0.80901699437, -0.58778525229, 0, 0, 0, 0, 0, 1;
    rt4 << 0, 0, -1, 0,  0.95105651629, 0.30901699437, 0, 0,  0.30901699437, -0.95105651629, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4f inv0, inv1, inv2, inv3, inv4;

    // 矩阵求逆
    inv0 = rt0.inverse();
    inv1 = rt1.inverse();
    inv2 = rt2.inverse();
    inv3 = rt3.inverse();
    inv4 = rt4.inverse();

    RT.push_back(rt0);
    RT.push_back(rt1);
    RT.push_back(rt2);
    RT.push_back(rt3);
    RT.push_back(rt4);

    INV.push_back(inv0);
    INV.push_back(inv1);
    INV.push_back(inv2);
    INV.push_back(inv3);
    INV.push_back(inv4);

    // 3 X 3 相机内参矩阵
    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type); // Intrisic matrix
    intrisicMat.at<double>(0, 0) = 476.715669286;
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 0;

    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = 476.715669286;
    intrisicMat.at<double>(2, 1) = 0;

    intrisicMat.at<double>(0, 2) = 400;
    intrisicMat.at<double>(1, 2) = 400;
    intrisicMat.at<double>(2, 2) = 1;

    // 雷达 -> 相机的旋转矩阵
    cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
    rVec.at<double>(0) = 0;
    rVec.at<double>(1) = 0;
    rVec.at<double>(2) = 0;

    // 雷达 -> 相机的平移矩阵
    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    tVec.at<double>(0) = 0.4;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = -0.1;

    // 畸变矩阵
    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);   // Distortion vector
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;
    distCoeffs.at<double>(4) = 0;

    std::cout << "Intrisic matrix: " << intrisicMat << std::endl << std::endl;
    std::cout << "Rotation vector: " << rVec << std::endl << std::endl;
    std::cout << "Translation vector: " << tVec << std::endl << std::endl;
    std::cout << "Distortion coef: " << distCoeffs << std::endl << std::endl;

    std::vector<cv::Point2d> image_points;
    std::vector<cv::Point2f> projected_points;

    // XYZRGB point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_t(new pcl::PointCloud<pcl::PointXYZRGB>);

    // XYZ point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int num = 2;
    int k = 0;

    ss.str("");
    ss << "/home/dlonng/Documents/Fusion/5_laser_camera_sim/data/cloud/cloud" << num << ".pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (ss.str(), *cloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return -1;
    }

    // PointXYZ -> Point3d
    std::vector<cv::Point3d> object_points = Generate3DPoints(cloud, k);

    cv::projectPoints(object_points, rVec, tVec, intrisicMat, distCoeffs, image_points);

    ss.str("");

    ss << "/home/dlonng/Documents/Fusion/5_laser_camera_sim/data/image/image" << num << "p" << k + 1 << ".jpg";

    cv::Mat img = cv::imread(ss.str(), CV_LOAD_IMAGE_COLOR);
    cv::Mat_<cv::Vec3b> img_ = img;

    cv::flip(img, img_, 1);

    for (int i = 0; i < image_points.size(); i++) {
        if (image_points[i].x >= 0 && image_points[i].x < 800 
            && image_points[i].y >= 0 && image_points[i].y < 800) {
            pcl::PointXYZRGB tmp_point;

            tmp_point.x = cloud->points[i].x;
            tmp_point.y = cloud->points[i].y;
            tmp_point.z = cloud->points[i].z;

            tmp_point.r = img_(round(image_points[i].x), round(image_points[i].y))[2];
            tmp_point.g = img_(round(image_points[i].x), round(image_points[i].y))[1];
            tmp_point.b = img_(round(image_points[i].x), round(image_points[i].y))[0];

            colored_cloud->points.push_back(tmp_point);
        }
    }

    pcl::transformPointCloud(*colored_cloud, *colored_cloud_t, INV[k]);
    colored_cloud->clear();

    // show colored point
    pcl::visualization::PCLVisualizer viewer("Cloud viewer");
    viewer.addPointCloud(colored_cloud, "sample cloud");
    viewer.setBackgroundColor(0, 0, 0);
    
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }


    return 0;

}



void InitMatrix() {

}



std::vector<cv::Point3d> Generate3DPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num) {
    std::vector<cv::Point3d> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f TR;
    TR << 0, 0, -1, 0,  0, 1, 0, 0,  1, 0, 0, 0, 0, 0, 0, 1;

    pcl::transformPointCloud(*cloud, *cloud_f, RT[num]);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_f);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 10);
    pass.filter(*cloud);

    std::cout << "size: " << cloud->size() << std::endl;

    for(int i = 0; i <= cloud->points.size(); i++) {
        points.push_back(cv::Point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }

    return points;
}
