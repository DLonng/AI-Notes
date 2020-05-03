/*
 * Description: ROS Node, Fusion img and point cloud
 * Author: Dlonng
 * Date: 2020-05-03 20:47:00
 * LastEditTime: 
 */


#include "lidar_camera_fusion.h"




LidarCameraFusion::LidarCameraFusion() {

    InitROS();
}



void LidarCameraFusion::InitROS() {
    std::string cloud_input;
    std::string image_input;
    std::string fusion_topic;

    // 是否需要修改？在 launch 文件中有主题参数的定义，需要搞清楚
    param_handle.param<std::string>("pointcloud_input", cloud_input, "/velodyne_points");
    param_handle.param<std::string>("image_input", image_input, "/cv_camera/image_raw");
    param_handle.param<std::string>("fusion_topic", fusion_topic, "/points_output");

    // 1 是什么意思？
    sub_cloud = topic_handle.subscribe(cloud_input, 1, &LidarCameraFusion::CloudCallback, this);
    
    sub_image = topic_handle.subscribe(image_input, 1, &LidarCameraFusion::ImageCallback, this);

    pub_fusion_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(fusion_topic, 1);
}



void LidarCameraFusion::ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg) {


}


void LidarCameraFusion::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {


}


tf::StampedTransform LidarCameraFusion::FindTransform(const std::string& target_frame, const std::string source_frame) {


}


pcl::PointXYZ LidarCameraFusion::TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform) {


}






