/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:47:00
 * @LastEditTime: 2020-06-17 09:12:00
 */

#include "lidar_camera_fusion.h"

#define USING_TF 0
#define USING_RAW 1
#define USING_KITTI 1

const std::string LidarCameraFusion::kNodeName = "lidar_camera_fusion";


LidarCameraFusion::LidarCameraFusion()
    : param_handle("~")
    , camera_lidar_tf_ok(false)
    , camera_instrinsics_mat_ok(false)
    , camera_extrinsic_mat_ok(false)
    , is_kitti(false)
    , image_frame_id("") {

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
void LidarCameraFusion::InitROS() {
    std::string calibration_file;
    param_handle.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty()) {
        ROS_ERROR("[%s]: missing calibration file path '%s'. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return ;
    }

    // 读取 Autoware 雷达相机外参标定文件
    cv::FileStorage fs(calibration_file, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_ERROR("[%s]: cannot open file calibration file [%s]. ", kNodeName.c_str(), calibration_file.c_str());
        ros::shutdown();
        return ;
    }

    // 导入雷达相机外参
    fs["CameraExtrinsicMat"] >> camera_extrinsic_mat;

    // 不用 TF 转换，需要对外参矩阵求逆，因为标定的方向是 [相机 -> 雷达]，而融合的方向是 [雷达 -> 相机]
    // 不需要对 KITTI 求逆，因为给的标定矩阵已经是 [雷达 -> 相机]
    param_handle.param<bool>("is_kitti", is_kitti, false);

    // 在 launch 中增加了一个是否使用 kitti 数据集的参数，主要是区分是否对外参矩阵求逆！
    if (is_kitti)
        camera_extrinsic_mat = camera_extrinsic_mat; // KITTI 不需要求逆
    else
        camera_extrinsic_mat = camera_extrinsic_mat.inv(); // 自己的小车如果不使用 TF 则需要求逆

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

    // 获取的参数名：image_input，获取的参数值存储在：image_input，缺省值：/camera/left/image_raw
    param_handle.param<std::string>("image_raw", image_raw, "/camera/left/image_raw");
    param_handle.param<std::string>("cloud_raw", cloud_raw, "/rslidar_points");

    param_handle.param<std::string>("semantic_img", semantic_img, "/semantic_img");
    param_handle.param<std::string>("semantic_confidence", semantic_confidence, "/semantic_confidence");
    
    param_handle.param<std::string>("semantic_cloud_max", semantic_cloud_max, "/semantic_cloud_max");

    // 订阅 image_input 话题
    // 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
    // 当处理消息速度不够时，可以调大第二个参数！
    // 收到订阅消息后调用 ImageCallback 处理图像 msg
    sub_image_raw = topic_handle.subscribe(image_raw, 1, &LidarCameraFusion::ImageRawCallback, this);
    sub_cloud_raw = topic_handle.subscribe(cloud_raw, 1, &LidarCameraFusion::CloudRawCallback, this);

    sub_semantic_img = topic_handle.subscribe(semantic_img, 1, &LidarCameraFusion::SemanticImageCallback, this);
    sub_confidence = topic_handle.subscribe(semantic_confidence, 1, &LidarCameraFusion::ConfidenceCallback, this);

    // 在 fusion_topic 上发布 sensor_msgs::PointCloud2 类型的消息
    // 第二个参数为缓冲区大小，缓冲区被占满后会丢弃先前发布的消息，目前为 1，可能需要更改！
    // 发布消息：pub_fusion_cloud.publish(msg)
    pub_semantic_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(semantic_cloud_max, 1);
}

/*
 * @Description: 使用相机内参矩阵，畸变矩阵和 OpenCV 对图像去畸变，并发布矫正后的图像消息
 * @Author: Dlonng
 * @Date: 2020-05-04
 * @LastEditTime:
 */
void LidarCameraFusion::ImageRawCallback(const sensor_msgs::Image::ConstPtr& image_msg) {
    // 1. 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // 2. ros_img -> cv_imag
    // image_msg: 图像指针，brg8: 编码参数
    // rgb8: CV_8UC3, color image with red-green-blue color order
    // https://blog.csdn.net/bigdog_1027/article/details/79090571
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");

    if (is_kitti) {
        // KITTI 已经去畸变
        image_frame = cv_image_ptr->image;
    } else {
        // cv_image: 原畸变 OpenCV 图像，image_frame：去畸变后的图像
        // camera_instrinsics：相机内参矩阵，distortion_coefficients：相机畸变矩阵
        // 我们自己做语义分割之前图像没有去畸变，这里调用 opencv undistort
        cv::undistort(cv_image_ptr->image, image_frame, camera_instrinsics_mat, distortion_coefficients);
    }


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
  * @details 
  *     image_raw: ZED Left
  *     semantic_img: LEDNet
  *     confidence: 暂时未提供
  * @param[in] cloud_msg 订阅的一帧原始点云
  * @return void
  * @note
  *     因为语义颜色的解析有 BUG，所以给 PointXYZRGBSemanticsMax 加上结构体作为 semantic_color 的 union
  *             struct {uint8_t s_b; uint8_t s_g; uint8_t s_r; uint8_t s_a;};
  * 
  *     目前没有使用 TF，直接读取外参矩阵
  *             
  * @todo 
  *     image_raw, cloud_raw, semantic_img, confidences 没有进行同步处理！
  *     confidence 没有增加，目前使用固定 0.8 作为测试！
  *     语义图像暂时没有加上 ID
  * @author DLonng
  * @date 2020-07-07
  */
void LidarCameraFusion::CloudRawCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // 确保当前融合的原始图像不为空
    if (image_frame.empty()) {
        ROS_INFO("[%s]: image_frame is empty! Waiting for current image frame ...", kNodeName.c_str());
        return ;
    }

    // 确保当前融合的语义图像不为空
    if (semantic_frame.empty()) {
        ROS_INFO("[%s]: semantic_frame is empty! Waiting for current semantic frame ...", kNodeName.c_str());
        return ;
    }

    // 确保当前融合的置信度矩阵不为空
    //if (confidences.empty()) {
    //    ROS_INFO("[%s]: confidence is empty! Waiting for current confidence frame ...", kNodeName.c_str());
    //    return ;
    //}

    // 再次确保 image_frame_id 不为空，因为 TF 要用到
    if (image_frame_id == "") {
        ROS_INFO("[%s]: image_frame_id is null! Please check image topic sender!", kNodeName.c_str());
        return ;
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

#if 0 
    pcl::PCLPointCloud2* tmp_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2::ConstPtr cloudPtr(tmp_cloud); 

    pcl::PCLPointCloud2 cloud_filtered;

    pcl_conversions::toPCL(*cloud_msg, *tmp_cloud);

    // 做一次离群点的滤波
    pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setMeanK(30);
    sor.setStddevMulThresh (1.0);
    sor.filter(cloud_filtered);

    sensor_msgs::PointCloud2 filter_output;
    pcl_conversions::fromPCL(cloud_filtered, filter_output);
#endif

    // 4. ROS point cloud -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);

    // void pcl::fromROSMsg(const sensor_msgs::PointCloud2 &, pcl::PointCloud<T> &);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud_msg);
    
    auto in_cloud_msg = pcl_cloud_msg;
    
    // 5. 需不需要对 PCL 点云做分割地面等操作？

    // 融合后的一帧点云
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<PointXYZRGBSemanticsMax>::Ptr out_cloud(new pcl::PointCloud<PointXYZRGBSemanticsMax>);
    out_cloud->points.clear();

    PointXYZRGBSemanticsMax semantic_point_max;

    std::vector<PointXYZRGBSemanticsMax> cam_cloud(in_cloud_msg->points.size());

    int row = 0;
    int col = 0;

    cv::Vec3b rgb_pixel;

    cv::Mat raw_point(4, 1, cv::DataType<double>::type);
    cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double tmp_z = 0.0;

    // 6. 遍历点云，融合像素
    for (size_t i = 0; i < in_cloud_msg->points.size(); i++) {
        /*
            坐标转换直接用矩阵相乘:
                T_lidar_cam(4 X 4) =
                                     R_lidar_cam(3 X 3), t_lidar_cam(1 X 3)
                                     0(1 X 3),           1

            点云到像素平面的投影矩阵 = 相机内参 * 畸变矩阵 * 雷达到相机的外参矩阵
                P_lidar_cam(3 X 4) = P_cam(3 X 3) * R_rect(3 X 4) * T_lidar_cam(4 X 4);

            坐标需要齐次化：
                [u, v, 1] = P_lidar_cam(3 X 4) * [x, y, z, 1];

            以上的矩阵使用 cv::Mat 定义即可
        */

#if USING_TF

       // //ROS_INFO("[%s]: in_cloud_msg x %f y %f z %f", kNodeName.c_str(), in_cloud_msg->points[i].x, in_cloud_msg->points[i].y, in_cloud_msg->points[i].z);
       // 
       // geometry_msgs::PointStamped rslidar_point; 
       // rslidar_point.header.frame_id = "rslidar";
       // 
       // rslidar_point.point.x = in_cloud_msg->points[i].x;
       // rslidar_point.point.y = in_cloud_msg->points[i].y;
       // rslidar_point.point.z = in_cloud_msg->points[i].z;

       // geometry_msgs::PointStamped left_point; 

       // transform_listener.transformPoint(image_frame_id, rslidar_point, left_point);
       // 
       // //ROS_INFO("[%s]:    cam_cloud x %f y %f z %f", kNodeName.c_str(), cam_cloud[i].x, cam_cloud[i].y, cam_cloud[i].z);
       // //ROS_INFO("[%s]:    left_cloud x %f y %f z %f", kNodeName.c_str(), left_point.point.x, left_point.point.y, left_point.point.z);
       // 
       // col = int(left_point.point.x * fx / left_point.point.z + cx);
       // row = int(left_point.point.y * fy / left_point.point.z + cy);
       // 
       // //ROS_INFO("[%s]: col %d row %d z %f", kNodeName.c_str(), col, row, left_point.point.z);
      

        cam_cloud[i] = TransformPoint(in_cloud_msg->points[i], camera_lidar_tf);

        col = int(cam_cloud[i].x * fx / cam_cloud[i].z + cx);
        row = int(cam_cloud[i].y * fy / cam_cloud[i].z + cy);

        tmp_z = cam_cloud[i].z;
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

        /*
                cv::Mat tmp_point(3, 1, cv::DataType<double>::type);
                tmp_point.at<double>(0, 0) = x;
                tmp_point.at<double>(1, 0) = y;
                tmp_point.at<double>(2, 0) = z;

                cv::Mat u_v(3, 1, cv::DataType<double>::type);
                u_v = camera_instrinsics_mat * tmp_point;

                int col = u_v.at<double>(0, 0) / u_v.at<double>(2, 0);
                int row = u_v.at<double>(1, 0) / u_v.at<double>(2, 0);
        */

        // 使用相机内参将三维空间点投影到像素平面
        col = int(x * fx / z + cx);
        row = int(y * fy / z + cy);

        tmp_z = z;
#endif
        
        // 只融合在像素平面内的点云, TF 没有效果是因为没改 z 的判断条件！
        if ((row >= 0) && (row < image_size.height) && (col >= 0) && (col < image_size.width) && (tmp_z > 0)) {
            // add XYZ
            semantic_point_max.x = in_cloud_msg->points[i].x;
            semantic_point_max.y = in_cloud_msg->points[i].y;
            semantic_point_max.z = in_cloud_msg->points[i].z;

            // ROS_INFO("[%s]: col %d row %d z %f", kNodeName.c_str(), col, row, left_point.point.z);
            
            // add RGB
            rgb_pixel = image_frame.at<cv::Vec3b>(row, col);
            semantic_point_max.r = rgb_pixel[2];
            semantic_point_max.g = rgb_pixel[1];
            semantic_point_max.b = rgb_pixel[0];

            // add semantic color
            cv::Vec3b semantic_pixel = semantic_frame.at<cv::Vec3b>(row, col);
            semantic_point_max.s_r = semantic_pixel[2];
            semantic_point_max.s_g = semantic_pixel[1];
            semantic_point_max.s_b = semantic_pixel[0];
            
            // This have bug!!!
            // 把 image 和 semantic 交换，image 仍然不能正常显示
            //uint8_t s_r = semantic_pixel[0];
            //uint8_t s_g = semantic_pixel[1];
            //uint8_t s_b = semantic_pixel[2];
            //semantic_point_max.semantic_color = 0;
            //semantic_point_max.semantic_color = (s_r << 16) + (s_g << 8) + s_b;

            // add confidence
            semantic_point_max.confidence = 0.8;
            //semantic_point_max.confidence = confidences.at<float>(row, col);

            out_cloud->points.push_back(semantic_point_max);
        }
    }

    // 执行欧拉聚类分割
    //std::vector<pcl::PointIndices> cluster_indices;

    // 聚类容忍度为 3m，最少点云数量为 100，最大点云数量为 25000，这 3 个参数需要根据实际情况调整！
    //EucCluster(out_cloud, cluster_indices, 3, 100, 25000);

    // 执行随机森林分类

    // 以下需要发布聚类 + 分割的误差最小化的融合点云

    sensor_msgs::PointCloud2 fusion_cloud;

    // PCL filter in here? must be ok

    // PCL cloud -> ROS cloud
    // void pcl::toROSMsg(const pcl::PointCloud<T> &, sensor_msgs::PointCloud2 &);
    pcl::toROSMsg(*out_cloud, fusion_cloud);

    fusion_cloud.header = cloud_msg->header;

    ROS_INFO("[%s]: publish fusion cloud.", kNodeName.c_str());

    // 发布融合后的带颜色的点云 
    pub_semantic_cloud.publish(fusion_cloud);
}


/**
  * @brief LEDNet 分割后图像的回调函数
  * @details 目前使用 Max Fusion 语义融合方法
  * @param[in] semantic_img 订阅的语义图像
  * @return void
  * @note 
  *     语义分割暂时没有去畸变
  *     语义图像暂时没有加上 ID
  * @todo 
  *     是否需要保存语义图像的 ID？
  *     是否需要保证语义图像与原始图像间的同步？
  * @author DLonng
  * @date 2020-07-07
  */
void LidarCameraFusion::SemanticImageCallback(const sensor_msgs::Image::ConstPtr& semantic_img) 
{
    // 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] SemanticImageCallback: wait to read camera instrinsics mat!", kNodeName.c_str());
        return;
    }

    // ros img -> cv img
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(semantic_img, "bgr8");

    // KITTI 数据集已经矫正过了，不需要去畸变
    if (is_kitti)
        semantic_frame = cv_image_ptr->image;
    else // 我们自己的语义分割没有去畸变，这里要处理
        cv::undistort(cv_image_ptr->image, semantic_frame, camera_instrinsics_mat, distortion_coefficients);
    
    // TODO: 是否需要单独保存语义图像的 ID，用来与 image_frame 比较确保是两者同步？
}

/**
  * @brief LEDNet 分割后最大概率语义图像的置信度回调
  * @details 目前使用 Max Fusion 语义融合方法
  * @param[in] confidence 订阅的置信度矩阵
  * @return void
  * @note 
  *     该函数的参数需要修改！
  * @todo 
  *     把参数修改为 float 类型的矩阵
  * @author DLonng
  * @date 2020-07-07
  */
void LidarCameraFusion::ConfidenceCallback(const sensor_msgs::Image::ConstPtr& confidence)
{
    // TODO: init confidences!

}

/*
 * @Description: 在 TF 树中寻找雷达和相机的坐标转换关系并返回
 * @Author: Dlonng
 * @Date: 2020-05-05
 * @LastEditTime:
 */
tf::StampedTransform LidarCameraFusion::FindTransform(const std::string& target_frame, const std::string& source_frame) {
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

/*
 * @Description: 对输入点云做 transform 坐标变换
 * @Author: Dlonng
 * @Date: 2020-05-05
 * @LastEditTime:
 */
pcl::PointXYZ LidarCameraFusion::TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform) {
    tf::Vector3 point(in_point.x, in_point.y, in_point.z);

    tf::Vector3 point_transform = in_transform * point;

    return pcl::PointXYZ(point_transform.x(), point_transform.y(), point_transform.z());
}

/*
 * @Description: 对融合后的点云做欧拉聚类分割
 * @Paramter
 *          in_cloud: 要聚类的点云
 *          cluster_tolerance: 聚类容忍度
 *          min_cluster_size: 聚类的最小点云数量
 *          max_cluster_size: 聚类的最大点云数量
 *          return: 聚类的结果 std::vector<pcl::PointIndices>，每行代表一个聚类簇，pcl::PointIndeices = std::vector<int>
 * @Author: DLonng
 * @Date: 2020-05-18
 * @LastEditTime: 2020-05-18
 */
void LidarCameraFusion::EucCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
                                   std::vector<pcl::PointIndices>& cluster_indices,
                                   int cluster_tolerance,
                                   int min_cluster_size,
                                   int max_cluster_size) {
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
