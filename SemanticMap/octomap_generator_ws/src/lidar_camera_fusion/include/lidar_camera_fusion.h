/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:41:00
 * @LastEditTime: 2020-06-17 09:12:00
 */

#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

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
//#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>

// system opecv-3.4.3
//#include <opencv2/opencv.hpp>

// ROS kinetic-3.3.1-dev
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>

//#include <image_transport/image_transport.h>

//#include <std_msgs/Float32MultiArray.h>
#include <rospy_tutorials/Floats.h>


struct PointXYZRGBSemanticsMax {
    PCL_ADD_POINT4D; // Preferred way of adding a XYZ+padding
    PCL_ADD_RGB;

    union // Semantic color
    {
        // 方便进行语义颜色的融合
        struct {
            uint8_t s_b;
            uint8_t s_g;
            uint8_t s_r;
            uint8_t s_a;
        };
        float semantic_color;
    };

    union // Confidences
    {
        float confidence;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned

} EIGEN_ALIGN16; // enforce SSE padding for correct memory alignment

// here we assume a XYZ + RGB + "sementic_color" + "confidence" (as fields)
// 注册自定义的点云类型
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZRGBSemanticsMax,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, semantic_color, semantic_color)(float, confidence, confidence))

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

    // 语义图像订阅回调函数
    void SemanticImageCallback(const sensor_msgs::Image::ConstPtr& semantic_img);

    // 语义置信度矩阵(Height x Width, Float32)订阅回调函数，这里需要接受哪种类型的参数
    //void ConfidenceCallback(const std_msgs::Float32MultiArray::ConstPtr& conf);
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
    ros::Publisher pub_semantic_cloud;

private:
    // 当前图像帧的 ID
    std::string image_frame_id;

    // 当前融合处理的原始图像
    cv::Mat image_frame;

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

    // 是否使用 KITTI 数据集测试
    bool is_kitti;

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

private:
    static const std::string kNodeName;
};

#endif // LIDAR_CAMERA_FUSION_H
