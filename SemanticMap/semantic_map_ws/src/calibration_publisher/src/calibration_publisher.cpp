#include "calibration_publisher.h"

const std::string kNodeName = "calibration_publisher";

static std::string camera_frame;
static std::string target_frame;

static cv::Mat cam_ext_mat;


void image_raw_callback(const sensor_msgs::Image& image_msg) {
    ros::Time image_time_stamp;
    image_time_stamp.sec = image_msg.header.stamp.sec;
    image_time_stamp.nsec = image_msg.header.stamp.nsec;

    TFRegistration(cam_ext_mat, image_time_stamp);
}

void TFRegistration(const cv::Mat& camera_ext_mat, const ros::Time& time_stamp) {
    tf::Matrix3x3 rotation_mat;
    tf::Quaternion quaternion;
    tf::Transform transfrom;

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    static tf::TransformBroadcaster broadcaster;

    rotation_mat.setValue(camera_ext_mat.at<double>(0, 0), camera_ext_mat.at<double>(0, 1), camera_ext_mat.at<double>(0, 2),
                          camera_ext_mat.at<double>(1, 0), camera_ext_mat.at<double>(1, 1), camera_ext_mat.at<double>(1, 2),
                          camera_ext_mat.at<double>(2, 0), camera_ext_mat.at<double>(2, 1), camera_ext_mat.at<double>(2, 2));

    rotation_mat.getRPY(roll, pitch, yaw, 1);

    quaternion.setRPY(roll, pitch, yaw);

    transfrom.setOrigin(tf::Vector3(camera_ext_mat.at<double>(0, 3),
                                    camera_ext_mat.at<double>(1, 3),
                                    camera_ext_mat.at<double>(2, 3)));

    transfrom.setRotation(quaternion);

    broadcaster.sendTransform(tf::StampedTransform(transfrom, time_stamp, target_frame, camera_frame));
    // 发送反向的 TF
    //broadcaster.sendTransform(tf::StampedTransform(transfrom, time_stamp, camera_frame, target_frame));
}






int main(int argc, char** argv) {
    ros::init(argc, argv, "calibration_publisher");

    ros::NodeHandle topic_handle;
    ros::NodeHandle param_handle("~");

    std::string image_topic;
    param_handle.param<std::string>("image_topic", image_topic, "/camera/left/image_raw");

    param_handle.param<std::string>("camera_frame", camera_frame, "left_frame");
    param_handle.param<std::string>("target_frame", target_frame, "rslidar");

    std::string calibration_file;
    param_handle.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty()) {
        ROS_ERROR("[%s]: missing calibration file path ''%s'. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return -1;
    }

    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_ERROR("[%s]: cannot open file calibration_file %s. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return -1;
    }


    fs["CameraExtrinsicMat"] >> cam_ext_mat;
    // 发送反向的 TF
    //cam_ext_mat = cam_ext_mat.inv();

    ROS_INFO("[%s]: camera_extrinsic_mat[0][0] %f", kNodeName.c_str(), cam_ext_mat.at<double>(0, 0));

    ros::Subscriber sub_imager = topic_handle.subscribe(image_topic, 10, &image_raw_callback);

    ros::spin();

    return 0;
}
