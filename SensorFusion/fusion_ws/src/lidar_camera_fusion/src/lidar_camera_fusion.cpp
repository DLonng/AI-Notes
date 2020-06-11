/*
 * @Description: ROS Node, Fusion img and point cloud
 * @Author: Dlonng
 * @Date: 2020-05-03 20:47:00
 * @LastEditTime:
 */

#include "lidar_camera_fusion.h"


const std::string LidarCameraFusion::kNodeName = "lidar_camera_fusion";


LidarCameraFusion::LidarCameraFusion()
    : param_handle("~")
    , camera_lidar_tf_ok(false)
    , camera_instrinsics_mat_ok(false)
    , camera_extrinsic_mat_ok(false)
    , image_frame_id("") {

    InitROS();
}

/*
 * @Description: 初始化当前节点，获取参数，订阅话题，发布话题
 * @Author: Dlonng
 * @Date: 2020-05-04
 * @LastEditTime:
 */
void LidarCameraFusion::InitROS() {
    std::string calibration_file;
    param_handle.param<std::string>("calibration_file", calibration_file, "");

    if (calibration_file.empty()) {
        ROS_ERROR("[%s]: missing calibration file path '%S'. ", kNodeName.c_str(), calibration_file.c_str());
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

    // 不用 TF 转换，需要对外参矩阵求逆，因为标定的方向是 [相机 -> 雷达]
    // 而融合的方向是 [雷达 -> 相机]
    camera_extrinsic_mat_inv = camera_extrinsic_mat.inv();
    camera_extrinsic_mat_ok = true;

    ROS_INFO("[%s]: camera_extrinsic_mat[0][0] %f", kNodeName.c_str(), camera_extrinsic_mat.at<double>(0, 0));

    // 读取相机内参
    fs["CameraMat"] >> camera_instrinsics_mat;
    camera_instrinsics_mat_ok = true;

    // 用于直接转换到像素平面
    fx = static_cast<float>(camera_instrinsics_mat.at<double>(0, 0));
    fy = static_cast<float>(camera_instrinsics_mat.at<double>(1, 1));
    cx = static_cast<float>(camera_instrinsics_mat.at<double>(0, 2));
    cy = static_cast<float>(camera_instrinsics_mat.at<double>(1, 2));

    ROS_INFO("[%s]: camera_instrinsics_mat fx %f fy %f cx %f cy %f", kNodeName.c_str(), fx, fy, cx, cy);

    // 畸变矩阵
    fs["DistCoeff"] >> distortion_coefficients;

    // 图像大小
    fs["ImageSize"] >> image_size;

    // 畸变模型
    fs["DistModel"] >> dist_model;

    std::string image_input;
    std::string cloud_input;
    std::string fusion_topic;

    // 获取的参数名：image_input，获取的参数值存储在：image_input，缺省值：/camera/left/image_raw
    param_handle.param<std::string>("image_input", image_input, "/camera/left/image_raw");
    param_handle.param<std::string>("cloud_input", cloud_input, "/rslidar_points");
    param_handle.param<std::string>("fusion_topic", fusion_topic, "/fusion_cloud");

    // 订阅 image_input 话题
    // 第二个参数是队列大小，以防我们处理消息的速度不够快，当缓存达到 1 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
    // 当处理消息速度不够时，可以调大第二个参数！
    // 收到订阅消息后调用 ImageCallback 处理图像 msg
    sub_image = topic_handle.subscribe(image_input, 1, &LidarCameraFusion::ImageCallback, this);

    sub_cloud = topic_handle.subscribe(cloud_input, 1, &LidarCameraFusion::CloudCallback, this);

    // 在 fusion_topic 上发布 sensor_msgs::PointCloud2 类型的消息
    // 第二个参数为缓冲区大小，缓冲区被占满后会丢弃先前发布的消息，目前为 1，可能需要更改！
    // 发布消息：pub_fusion_cloud.publish(msg)
    pub_fusion_cloud = topic_handle.advertise<sensor_msgs::PointCloud2>(fusion_topic, 1);
}

/*
 * @Description: 使用相机内参矩阵，畸变矩阵和 OpenCV 对图像去畸变，并发布矫正后的图像消息
 * @Author: Dlonng
 * @Date: 2020-05-04
 * @LastEditTime:
 */
void LidarCameraFusion::ImageCallback(const sensor_msgs::Image::ConstPtr& image_msg) {
    // 1. 确保相机内参和畸变矩阵已经初始化！
    if (camera_instrinsics_mat_ok == false) {
        ROS_INFO("[%s] wait to read camera instrinsics mat!", "lidar_camera_fusion");
        return;
    }

    // 2. ros_img -> cv_imag
    // image_msg: 图像指针，brg8: 编码参数
    // rgb8: CV_8UC3, color image with red-green-blue color order
    // https://blog.csdn.net/bigdog_1027/article/details/79090571
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    cv::Mat cv_image = cv_image_ptr->image;

    // 3. OpenCV 去畸变
    // cv_image: 原畸变 OpenCV 图像，image_frame：去畸变后的图像
    // camera_instrinsics：相机内参矩阵，distortion_coefficients：相机畸变矩阵
    cv::undistort(cv_image, image_frame, camera_instrinsics_mat, distortion_coefficients);

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

/*
 * @Description: 融合点云和图像
 * @Author: Dlonng
 * @Date: 2020-05-04
 * @LastEditTime:
 */
void LidarCameraFusion::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // 1. 确保当前融合的图像已经去畸变
    if (image_frame.empty() || image_frame_id == "") {
        ROS_INFO("[%s]: waiting for current image frame ...", kNodeName.c_str());
        return;
    }

#if USING_TF

    // 2. 从 tf 树中寻找雷达和相机的坐标变换关系
    if (camera_lidar_tf_ok == false)
        camera_lidar_tf = FindTransform(image_frame_id, cloud_msg->header.frame_id);

    // 3. 保证相机内参和坐标转换准备完成
    if (camera_instrinsics_mat_ok == false || camera_lidar_tf_ok == false) {
        ROS_INFO("[%s]: waiting for camera intrinsics and camera lidar tf ...", kNodeName.c_str());
        return;
    }

#else

    // 3. 保证相机内参和坐标转换准备完成
    if (camera_instrinsics_mat_ok == false || camera_extrinsic_mat_ok == false) {
        ROS_INFO("[%s]: waiting for camera_instrinsics_mat and camera_extrinsic_mat...", kNodeName.c_str());
        return;
    }

#endif

    // 4. ROS point cloud -> PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_msg(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl_ros
    pcl::fromROSMsg(*cloud_msg, *in_cloud_msg);

    // 5. 需不需要对 PCL 点云做分割地面等操作？

    // 融合后的一帧点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    out_cloud->points.clear();

    pcl::PointXYZRGB color_point;

    std::vector<pcl::PointXYZ> cam_cloud(in_cloud_msg->points.size());

    int row = 0;
    int col = 0;

    cv::Vec3b rgb_pixel;

    cv::Mat raw_point(4, 1, cv::DataType<double>::type);
    cv::Mat transformed_point(4, 1, cv::DataType<double>::type);

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double tmp_z = 0.0;

    // 6. 遍历点云，给每个点加上颜色
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
        transformed_point = camera_extrinsic_mat_inv * raw_point;

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
            color_point.x = in_cloud_msg->points[i].x;
            color_point.y = in_cloud_msg->points[i].y;
            color_point.z = in_cloud_msg->points[i].z;

            //ROS_INFO("[%s]: col %d row %d z %f", kNodeName.c_str(), col, row, left_point.point.z);
            
            rgb_pixel = image_frame.at<cv::Vec3b>(row, col);

            color_point.r = rgb_pixel[2];
            color_point.g = rgb_pixel[1];
            color_point.b = rgb_pixel[0];

            out_cloud->points.push_back(color_point);
        }
    }

    // 执行欧拉聚类分割
    //std::vector<pcl::PointIndices> cluster_indices;

    // 聚类容忍度为 3m，最少点云数量为 100，最大点云数量为 25000，这 3 个参数需要根据实际情况调整！
    //EucCluster(out_cloud, cluster_indices, 3, 100, 25000);

    // 执行随机森林分类

    // 以下需要发布聚类 + 分割的误差最小化的融合点云

    sensor_msgs::PointCloud2 fusion_cloud;

    // PCL cloud -> ROS cloud
    pcl::toROSMsg(*out_cloud, fusion_cloud);

    fusion_cloud.header = cloud_msg->header;
    //fusion_cloud.header.frame_id = "camera";

    //ROS_INFO("[%s]: cloud_frame_id %s", kNodeName.c_str(), cloud_msg->header.frame_id.c_str());

    //ROS_INFO("lidar_camera_fusion: publish fusion cloud");

    // 发布融合后的带颜色的点云
    pub_fusion_cloud.publish(fusion_cloud);
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
        // 寻找反向 TF
        //transform_listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);
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
 * @Author: Dlonng
 * @Date: 2020-05-18
 * @LastEditTime:
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
