#ifndef CALIBRATION_PUBLISHER
#define CALIBRATION_PUBLISHER


#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


void image_raw_callback(const sensor_msgs::Image& image_msg);

void TFRegistration(const cv::Mat& camera_ext_mat, const ros::Time& time_stamp);


























#endif //CALIBRATION_PUBLISHER

