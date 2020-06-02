/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:41:00
 * @LastEditTime: 
 */

#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>

class LidarCameraFusion {
public:
    LidarCameraFusion();

private:
    // 初始化 ROS，订阅主题，设置参数等
    void InitROS();

    // 图像订阅回调函数
    void ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

    // 点云订阅回调函数
    void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    // 从 tf 树中寻找两个 frame_id 之间的变换关系，第二个参数应该也可以用引用传递
    tf::StampedTransform FindTransform(const std::string& target_frame, const std::string source_frame);

    // 将 in_point 利用 in_transform 进行坐标转换
    pcl::PointXYZ TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform);

    // 对融合后的点云执行欧拉聚类分割
    void LidarCameraFusion::EucCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, 
                                       std::vector<pcl::PointIndices>& cluster_indices, 
                                       int cluster_tolerance, 
                                       int min_cluster_size, 
                                       int max_cluster_size);

private:
    // 为何要分为 2 个呢？只用一个行不行？
    ros::NodeHandle param_handle;

    ros::NodeHandle topic_handle;

    // 图像订阅者
    ros::Subscriber sub_image;

    // 点云订阅者
    ros::Subscriber sub_cloud;

    // 融合结果发布者
    ros::Publisher pub_fusion_cloud;

private:
    // 当前图像帧的 ID
    std::string image_frame_id;

    // 当前融合处理的图像
    cv::Mat image_frame;

    // ZED 相机内参，应该要两个变量
    cv::Mat camera_instrinsics;

    // ZED 相机畸变矩阵
    cv::Mat distortion_coefficients;

    cv::Size image_size;

private:
    // 定义相机和雷达之间的坐标转换关系
    tf::StampedTransform camera_lidar_tf;

    // tf 坐标监听者
    tf::TransformListener transform_listener;

    // 判断是否找到雷达和相机之间的坐标转换关系
    bool camera_lidar_tf_ok;

private:
    // 融合的带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
};

#endif // LIDAR_CAMERA_FUSION_H
