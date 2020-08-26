/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:47:00
 * @LastEditTime: 2020-06-17 09:12:00
 */

#include "lidar_camera_fusion_no_msg.h"

#define USING_TF 0

// SWITCH_FUSION 设置为 0 才使用下面这 2 个开关
// USING_MAX_SEMANTIC 0 表示在点云回调中使用 Max 类型的点云，但是不給语义信息
#define USING_MAX_SEMANTIC 0

const std::string LidarCameraFusion::kNodeName = "lidar_camera_fusion";

// 1: max semantic, 2: bayes semantic
const int LidarCameraFusion::kMaxSemanticType = 1;
const int LidarCameraFusion::kBayesSemanticType = 2;

LidarCameraFusion::LidarCameraFusion()
    : param_handle("~")
    , camera_lidar_tf_ok(false)
    , camera_instrinsics_mat_ok(false)
    , camera_extrinsic_mat_ok(false)
    , image_frame_id("")
    , semantic_type(kMaxSemanticType)
{

    InitROS();
}

/**
  * @brief 函数简要说明
  * @details 函数细节说明
  * @param[in] 参数名 参数描述
  * @param[out] 参数名 参数描述
  * @return pointer to the updated NODE
  * @note 注意事项
  * @todo 需要去实现的功能
  * @author 作者
  * @date 日期
  */
void LidarCameraFusion::InitROS()
{
    param_handle.param<int>("semantic_type", semantic_type, kMaxSemanticType);

    std::string calibration_file;
    param_handle.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty()) {
        ROS_ERROR("[%s]: missing calibration file path '%s'. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return;
    }

    // 读取 Autoware 雷达相机外参标定文件
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_ERROR("[%s]: cannot open file calibration file [%s]. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return;
    }

    // 导入雷达相机外参
    fs["CameraExtrinsicMat"] >> camera_extrinsic_mat;

    // 自己的小车如果不使用 TF 则需要求逆
    camera_extrinsic_mat = camera_extrinsic_mat.inv(); 
    camera_extrinsic_mat_ok = true;

    ROS_INFO("[%s]: read camera_extrinsic_mat[0][0] %f", kNodeName.c_str(), camera_extrinsic_mat.at<double>(0, 0));

    // 读取相机内参，相机内参也可以考虑订阅 CameraInfo
    fs["CameraMat"] >> camera_instrinsics_mat;
    camera_instrinsics_mat_ok = true;

    // 用于直接转换到像素平面
    fx = static_cast<float>(camera_instrinsics_mat.at<double>(0, 0));
    fy = static_cast<float>(camera_instrinsics_mat.at<double>(1, 1));
    cx = static_cast<float>(camera_instrinsics_mat.at<double>(0, 2));
    cy = static_cast<float>(camera_instrinsics_mat.at<double>(1, 2));

    ROS_INFO("[%s]: read camera_instrinsics_mat fx %f fy %f cx %f cy %f", kNodeName.c_str(), fx, fy, cx, cy);

    // 图像大小
    fs["ImageSize"] >> image_size;

    // 初始化置信度矩阵
    confidences = cv::Mat::zeros(image_size.height, image_size.width, cv::DataType<float>::type);

    // 畸变矩阵
    fs["DistCoeff"] >> distortion_coefficients;

    // 畸变模型
    fs["DistModel"] >> dist_model;

    // 原始 ZED left 图像
    std::string image_raw;
    // 原始 Robosense-16 点云
    std::string cloud_raw;

    // LEDNet 分割后的语义图像
    std::string semantic_img;

    // LEDNet 语义图像对应的置信度矩阵
    std::string semantic_confidence;

    // Max Fusion 使用的语义点云类型
    std::string semantic_cloud_max;

    // Bayes Fusion 使用的语义点云类型
    std::string semantic_cloud_bayes;

    // max 概率语义消息
    std::string max_semantic;

    // bayes 语义消息
    std::string bayes_semantic;

    // 获取的参数名：image_input，获取的参数值存储在：image_input，缺省值：/camera/left/image_raw
    param_handle.param<std::string>("image_raw", image_raw, "/camera/left/image_raw");
    param_handle.param<std::string>("cloud_raw", cloud_raw, "/rslidar_points");

    param_handle.param<std::string>("semantic_img", semantic_img, "/semantic_img");
    param_handle.param<std::string>("semantic_confidence", semantic_confidence, "/semantic_array");

    param_handle.param<std::string>("semantic_cloud_max", semantic_cloud_max, "/semantic_cloud_max");
    param_handle.param<std::string>("semantic_cloud_bayes", semantic_cloud_bayes, "/semantic_cloud_bayes");

    // 订阅 image_input 话题
    // 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
    // 当处理消息速度不够时，可以调大第二个参数！
    // 收到订阅消息后调用 ImageCallback 处理图像 msg
    int topic_buff = 5;
    sub_image_raw = topic_handle.subscribe(image_raw, topic_buff, &LidarCameraFusion::ImageRawCallback, this);
    sub_cloud_raw = topic_handle.subscribe(cloud_raw, topic_buff, &LidarCameraFusion::CloudRawCallback, this);

    // 最大概率语义图像和置信度分开订阅的逻辑
    sub_semantic_img = topic_handle.subscribe(semantic_img, topic_buff, &LidarCameraFusion::SemanticImageCallback, this);
    sub_confidence = topic_handle.subscribe(semantic_confidence, topic_buff, &LidarCameraFusion::ConfidenceCallback, this);

    // 在 fusion_topic 上发布 sensor_msgs::PointCloud2 类型的消息
    // 第二个参数为缓冲区大小，缓冲区被占满后会丢弃先前发布的消息，目前为 1，可能需要更改！
    // 发布消息：pub_fusion_cloud.publish(msg)
    pub_max_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_max, topic_buff);
    pub_bayes_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_bayes, topic_buff);
}

/**
  * @brief ZED 左相机图像订阅回调函数
  * @details 相机目前的发送频率是 20 Hz
  * @param[in]  
  * @return void
  * @note
  *     注意不同数据集的去畸变问题
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::ImageRawCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // ros_img -> cv_imag
    // image_msg: 图像指针，brg8: 编码参数
    // rgb8: CV_8UC3, color image with red-green-blue color order
    // https://blog.csdn.net/bigdog_1027/article/details/79090571
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");

    // cv_image: 原畸变 OpenCV 图像，image_frame：去畸变后的图像
    // camera_instrinsics：相机内参矩阵，distortion_coefficients：相机畸变矩阵
    // 我们自己做语义分割之前图像没有去畸变，这里调用 opencv undistort
    cv::undistort(cv_image_ptr->image, image_frame, camera_instrinsics_mat, distortion_coefficients);

#if 0
    // 4. 发布去畸变的图像消息
    static image_transport::ImageTransport it(topic_handle);
    static image_transport::Publisher pub_image = it.advertise("identified_image", 1);
    static sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_frame).toImageMsg();
    pub_image.publish(msg);
#endif

    // 5. 保存当前图像帧的 ID 和大小
    image_frame_id = image_msg->header.frame_id;
    image_size.height = image_frame.rows;
    image_size.width = image_frame.cols;

    //ROS_INFO("[%s]: image_frame_id %s", kNodeName.c_str(), image_frame_id.c_str());
}

/**
  * @brief  融合 image_raw, cloud_raw, semantic_img, confidences 为一帧语义点云
  * @details 雷达目前的发送频率是 10 Hz
  * @param[in] cloud_msg 订阅的一帧原始点云
  * @return void
  * @note
  *     1. 因为语义颜色的解析有 BUG，所以给 PointXYZRGBSemanticsMax 加上结构体作为 semantic_color 的 union
  *             struct {uint8_t s_b; uint8_t s_g; uint8_t s_r; uint8_t s_a;};
  * 
  *     2. 给 Bayes 结构体也加上了 RGBA 的 union struct
  * 
  *     3. 目前没有使用 TF，直接读取外参矩阵
  *     
  *     4. 目前订阅 2 组语义分割的 Bayes 语义后的发布频率为 5 Hz左右，与语义分割的发布频率几乎相同
  * @todo 
  *     image_raw, cloud_raw, semantic_img, confidences 没有进行同步处理！
  *     语义图像暂时没有加上 ID
  *     测试自定义 semantic/max_msg 消息！
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::CloudRawCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // 确保当前融合的原始图像不为空
    if (image_frame.empty()) {
        ROS_INFO("[%s]: image_frame is empty! Waiting for current image frame ...", kNodeName.c_str());
        return;
    }

    // 再次确保 image_frame_id 不为空，因为 TF 要用到
    if (image_frame_id == "") {
        ROS_INFO("[%s]: image_frame_id is null! Please check image topic sender!", kNodeName.c_str());
        return;
    }

#if USING_TF

    // 从 tf 树中寻找雷达和相机的坐标变换关系
    if (camera_lidar_tf_ok == false)
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);

    // 保证相机内参和坐标转换准备完成
    if (camera_instrinsics_mat_ok == false || camera_lidar_tf_ok == false) {
        ROS_INFO("[%s]: waiting for camera intrinsics and camera lidar tf ...", kNodeName.c_str());
        return;
    }

#else
    // 直接使用读取的外参矩阵
    // 保证相机内参和坐标转换读取成功
    if (camera_instrinsics_mat_ok == false || camera_extrinsic_mat_ok == false) {
        ROS_INFO("[%s]: waiting for camera_instrinsics_mat and camera_extrinsic_mat...", kNodeName.c_str());
        return;
    }
#endif

    // 4. ROS point cloud -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

    // void pcl::fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud_msg);

    auto in_cloud_msg = pcl_cloud_msg;

    int row = 0;
    int col = 0;

    cv::Vec3b rgb_pixel;

    cv::Mat raw_point(4, 1, cv::DataType<double>::type);
    cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double tmp_z = 0.0;

    if (kMaxSemanticType == semantic_type) {
#if USING_MAX_SEMANTIC
        if (semantic_frame.empty()) {
            ROS_INFO("[%s]: semantic_frame is empty! Waiting for current semantic frame ...", kNodeName.c_str());
            return;
        }

        if (confidences.empty()) {
            ROS_INFO("[%s]: confidence is empty! Waiting for current confidence frame ...", kNodeName.c_str());
            return;
        }
#endif        

        // 初始化最大概率语义点云
        pcl::PointCloud<PointXYZRGBSemanticsMax>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsMax>);
        out_cloud->points.clear();

        PointXYZRGBSemanticsMax semantic_point_max;
        std::vector<PointXYZRGBSemanticsMax> cam_cloud(in_cloud_msg->points.size());

        // 6. 遍历点云，融合像素
        for (size_t i = 0; i < in_cloud_msg->points.size(); i++) {
#if USING_TF

#else
            // 用 RT 矩阵变换
            raw_point.at<double>(0, 0) = in_cloud_msg->points[i].x;
            raw_point.at<double>(1, 0) = in_cloud_msg->points[i].y;
            raw_point.at<double>(2, 0) = in_cloud_msg->points[i].z;
            raw_point.at<double>(3, 0) = 1;

            // 4 X 1 = 4 X 4 * 4 X 1;
            transformed_point = camera_extrinsic_mat * raw_point;

            x = transformed_point.at<double>(0, 0);
            y = transformed_point.at<double>(1, 0);
            z = transformed_point.at<double>(2, 0);
            // [3][0] = 1;

            // 使用相机内参将三维空间点投影到像素平面
            col = int(x * fx / z + cx);
            row = int(y * fy / z + cy);

            tmp_z = z;
#endif
            if ((row >= 0) && (row < image_size.height) && (col >= 0) && (col < image_size.width) && (tmp_z > 0)) {

                semantic_point_max.x = in_cloud_msg->points[i].x;
                semantic_point_max.y = in_cloud_msg->points[i].y;
                semantic_point_max.z = in_cloud_msg->points[i].z;

                rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
                semantic_point_max.r = rgb_pixel[2];
                semantic_point_max.g = rgb_pixel[1];
                semantic_point_max.b = rgb_pixel[0];

#if USING_MAX_SEMANTIC
                cv::Vec3b semantic_pixel = semantic_frame.at<cv::Vec3b>(row, col);
                semantic_point_max.confidence = confidences.at<float>(row, col);
#else
                cv::Vec3b semantic_pixel = image_frame.at<cv::Vec3b>(row, col);
                semantic_point_max.confidence = 0.8;
#endif
                semantic_point_max.s_r = semantic_pixel[2];
                semantic_point_max.s_g = semantic_pixel[1];
                semantic_point_max.s_b = semantic_pixel[0];

                out_cloud->points.push_back(semantic_point_max);
            }
        }

        sensor_msgs::PointCloud2 max_semantic_cloud;
        pcl::toROSMsg(*out_cloud, max_semantic_cloud);
        max_semantic_cloud.header = cloud_msg->header;

        ROS_INFO("[%s]: [%s] publish max_semantic_cloud.", kNodeName.c_str(), __FUNCTION__);

        pub_max_semantic_cloud.publish(max_semantic_cloud);

    } else {
        // 初始化 Bayes 概率语义点云
        pcl::PointCloud<PointXYZRGBSemanticsBayesian>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsBayesian>);
        out_cloud->points.clear();

        PointXYZRGBSemanticsBayesian semantic_point_bayes;
        std::vector<PointXYZRGBSemanticsBayesian> cam_cloud(in_cloud_msg->points.size());

        for (size_t i = 0; i < in_cloud_msg->points.size(); i++) {
#if USING_TF

#else
            // 用 RT 矩阵变换
            raw_point.at<double>(0, 0) = in_cloud_msg->points[i].x;
            raw_point.at<double>(1, 0) = in_cloud_msg->points[i].y;
            raw_point.at<double>(2, 0) = in_cloud_msg->points[i].z;
            raw_point.at<double>(3, 0) = 1;

            // 4 X 1 = 4 X 4 * 4 X 1;
            transformed_point = camera_extrinsic_mat * raw_point;

            x = transformed_point.at<double>(0, 0);
            y = transformed_point.at<double>(1, 0);
            z = transformed_point.at<double>(2, 0);
            // [3][0] = 1;

            // 使用相机内参将三维空间点投影到像素平面
            col = int(x * fx / z + cx);
            row = int(y * fy / z + cy);

            tmp_z = z;
#endif

            if ((row >= 0) && (row < image_size.height) && (col >= 0) && (col < image_size.width) && (tmp_z > 0)) {
                // XYZ
                semantic_point_bayes.x = in_cloud_msg->points[i].x;
                semantic_point_bayes.y = in_cloud_msg->points[i].y;
                semantic_point_bayes.z = in_cloud_msg->points[i].z;

                // RGB
                rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
                semantic_point_bayes.r = rgb_pixel[2];
                semantic_point_bayes.g = rgb_pixel[1];
                semantic_point_bayes.b = rgb_pixel[0];

                // semantic 1
                cv::Vec3b semantic_pixel_1 = image_frame.at<cv::Vec3b>(row, col);
                semantic_point_bayes.s1_r = semantic_pixel_1[2];
                semantic_point_bayes.s1_g = semantic_pixel_1[1];
                semantic_point_bayes.s1_b = semantic_pixel_1[0];

                // semantic 2
                cv::Vec3b semantic_pixel_2 = image_frame.at<cv::Vec3b>(row, col);
                semantic_point_bayes.s2_r = semantic_pixel_2[2];
                semantic_point_bayes.s2_g = semantic_pixel_2[1];
                semantic_point_bayes.s2_b = semantic_pixel_2[0];
                // semantic 3 no use!

                // confidence 1 and 2
                //semantic_point_bayes.confidence1 = bayes_confidences_1.at<float>(row, col);
                //semantic_point_bayes.confidence2 = bayes_confidences_2.at<float>(row, col);
                semantic_point_bayes.confidence1 = 0.8;
                semantic_point_bayes.confidence2 = 0.8;
                // confidence 3 no use!

                out_cloud->points.push_back(semantic_point_bayes);
            }
        }

        sensor_msgs::PointCloud2 bayes_semantic_cloud;
        pcl::toROSMsg(*out_cloud, bayes_semantic_cloud);
        bayes_semantic_cloud.header = cloud_msg->header;

        ROS_INFO("[%s]: [%s] publish bayes_semantic_cloud.", kNodeName.c_str(), __FUNCTION__);

        pub_bayes_semantic_cloud.publish(bayes_semantic_cloud);
    }
}

/**
  * @brief 语义分割的最大概率语义图像回调
  * @details 没有使用 semantic_msg/max_msg
  * @param[in] semantic_img 订阅的语义图像
  * @return void
  * @note 
  *     语义分割暂时没有去畸变
  *     语义图像暂时没有加上 ID
  * @todo 
  *     是否需要保存语义图像的 ID？
  *     是否需要保证语义图像与原始图像间的同步？
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::SemanticImageCallback(const sensor_msgs::Image::ConstPtr& semantic_img)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] SemanticImageCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(semantic_img, "bgr8");

    cv::undistort(cv_image_ptr->image, semantic_frame, camera_instrinsics_mat, distortion_coefficients);
}

/**
  * @brief 语义分割的最大概率置信度回调
  * @details 没有使用 semantic_msg/max_msg
  * @param[in] conf 订阅的置信度矩阵
  * @return void
  * @note 
  * @author DLonng
  * @date 2020-07-24
  */
void LidarCameraFusion::ConfidenceCallback(const rospy_tutorials::Floats::ConstPtr& conf)
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] SemanticImageCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // TODO: init confidences!
    for (int r = 0; r < image_size.height; r++) {
        for (int l = 0; l < image_size.width; l++) {
            this->confidences.at<float>(r, l) = conf->data[r * image_size.width + l];
        }
    }
}

tf::StampedTransform LidarCameraFusion::FindTransform(const std::string& target_frame, const std::string& source_frame)
{
    tf::StampedTransform transform;

    camera_lidar_tf_ok = false;

    try {
        transform_listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        camera_lidar_tf_ok = true;
        ROS_INFO("[%s]: camera_lidar_tf Get!", kNodeName.c_str());
    } catch (tf::TransformException e) {
        ROS_INFO("[%s]: %s", kNodeName.c_str(), e.what());
    }

    return transform;
}

pcl::PointXYZ LidarCameraFusion::TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform)
{
    tf::Vector3 point(in_point.x, in_point.y, in_point.z);

    tf::Vector3 point_transform = in_transform * point;

    return pcl::PointXYZ(point_transform.x(), point_transform.y(), point_transform.z());
}

/**
 * @brief: 对融合后的点云做欧拉聚类分割
 * @param[in]: in_cloud 要聚类的点云
 * @param[in]: cluster_tolerance 聚类容忍度
 * @param[in]: min_cluster_size 聚类的最小点云数量
 * @param[in]: max_cluster_size 聚类的最大点云数量
 * @return: 聚类的结果 std::vector<pcl::PointIndices>，每行代表一个聚类簇，pcl::PointIndeices = std::vector<int>
 * @author: DLonng
 * @todo:
 *      聚类工作还未开始！
 * @date: 2020-07-24
 */
void LidarCameraFusion::EucCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
    std::vector<pcl::PointIndices>& cluster_indices,
    int cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size)
{
    // 设置使用 kdtree 搜索
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kd_tree->setInputCloud(in_cloud);

    // 创建欧拉聚类对象
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euc;

    // 设置聚类参数
    euc.setClusterTolerance(cluster_tolerance);
    euc.setMinClusterSize(min_cluster_size);
    euc.setMaxClusterSize(max_cluster_size);

    // 设置使用 kdtree 搜索最近邻居
    euc.setSearchMethod(kd_tree);

    euc.setInputCloud(in_cloud);

    // 执行欧拉聚类
    euc.extract(cluster_indices);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "lidar_camera_fusion_node");

    LidarCameraFusion fusion_node;

    ros::spin();

    return 0;
}