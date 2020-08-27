/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:41:00
 * @LastEditTime: 2020-06-17 09:12:00
 */

#ifndef LIDAR_CAMERA_FUSION_NO_MSG_H
#define LIDAR_CAMERA_FUSION_NO_MSG_H

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

#include <opencv-3.3.1-dev/opencv2/opencv.hpp>

#include <rospy_tutorials/Floats.h>

#include "semantics_point_type.h"

class LidarCameraFusion {
public:
    LidarCameraFusion();

private:
    // 初始化 ROS，订阅主题，设置参数等
    void InitROS();

    // 图像订阅回调函数
    void ImageRawCallback(const sensor_msgs::Image::ConstPtr& image_msg);

    // 点云订阅回调函数
    void CloudRawCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    // Max 语义图像订阅回调函数
    void SemanticImageCallback(const sensor_msgs::Image::ConstPtr& semantic_img);

    // Max 语义图像置信度回调
    void ConfidenceCallback(const rospy_tutorials::Floats::ConstPtr& conf);

    // 从 tf 树中寻找两个 frame_id 之间的变换关系，第二个参数应该也可以用引用传递
    tf::StampedTransform FindTransform(const std::string& target_frame, const std::string& source_frame);

    // 将 in_point 利用 in_transform 进行坐标转换
    pcl::PointXYZ TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform);

    // 对融合后的点云执行欧拉聚类分割
    void EucCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
        std::vector<pcl::PointIndices>& cluster_indices,
        int cluster_tolerance,
        int min_cluster_size,
        int max_cluster_size);

private:
    ros::NodeHandle param_handle;

    ros::NodeHandle topic_handle;

    // 图像订阅者
    ros::Subscriber sub_image_raw;

    // 点云订阅者
    ros::Subscriber sub_cloud_raw;

    // 语义图像订阅者
    ros::Subscriber sub_semantic_img;

    // 语义图像置信度
    ros::Subscriber sub_confidence;

    // 融合结果发布者
    ros::Publisher pub_bayes_semantic_cloud;
    ros::Publisher pub_max_semantic_cloud;

private:
    // 当前图像帧的 ID
    std::string image_frame_id;

    // 当前融合处理的原始图像
    cv::Mat image_frame;

    sensor_msgs::PointCloud2 cloud_frame;

    // 当前融合处理的语义图像
    cv::Mat semantic_frame;

    // Robosense 雷达和 ZED 相机外参
    cv::Mat camera_extrinsic_mat;

    // 外参逆矩阵
    cv::Mat camera_extrinsic_mat_inv;

    // ZED 相机内参
    cv::Mat camera_instrinsics_mat;

    // ZED 相机内参
    float fx, fy, cx, cy;

    // ZED 相机畸变矩阵
    cv::Mat distortion_coefficients;

    // ZED 相机图像大小
    cv::Size image_size;

    // ZED 相机畸变模型
    std::string dist_model;

    // 语义图像置信度矩阵
    cv::Mat confidences;

    // 内参是否初始化
    bool camera_instrinsics_mat_ok;

    // 雷达相机外参是否初始化
    bool camera_extrinsic_mat_ok;

    // 语义点云类型
    int semantic_type;

private:
    // Max 语义
    cv::Mat max_frame;
    cv::Mat max_confidences;

private:
    // 定义相机和雷达之间的坐标转换关系
    tf::StampedTransform camera_lidar_tf;

    // tf 坐标监听者
    tf::TransformListener transform_listener;

    // 判断是否找到雷达和相机之间的坐标转换关系
    bool camera_lidar_tf_ok;

private:
    static const std::string kNodeName;
    static const int kMaxSemanticType;
    static const int kBayesSemanticType;
};

#endif // LIDAR_CAMERA_FUSION_NO_MSG_H
