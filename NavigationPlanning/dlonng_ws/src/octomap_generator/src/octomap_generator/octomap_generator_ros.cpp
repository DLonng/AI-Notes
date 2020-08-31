
#include <octomap_generator/octomap_generator_ros.h>

/*
#include <cmath>
#include <cstring>
#include <sstream>


#include <octomap_msgs/conversions.h>

#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>

// 放这里会报错，节点启动失败
//#include <pcl/segmentation/sac_segmentation.h>
*/

#define TEST_VEL 0

const std::string OctomapGeneratorNode::kNodeName = "OctomapGenerator";

OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& nh)
    : nh_(nh)
{
    nh_.getParam("/octomap/tree_type", tree_type_);

    // Initiate octree
    if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN || tree_type_ == SEMANTICS_OCTREE_MAX) {
        if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN) {
            ROS_INFO("[%s]: [%s] Semantic octomap generator [bayesian fusion]", kNodeName.c_str(), __FUNCTION__);
            octomap_generator_ = new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
            // 创建局部 Bayes 地图对象
            // Bayes 语义融合的发布频率问题还没解决
            local_octomap_generator = new OctomapGenerator<PCLSemanticsBayesian, LocalSemanticsOctreeBayesian>();
        } else {
            ROS_INFO("[%s]: [%s] Semantic octomap generator [max fusion]", kNodeName.c_str(), __FUNCTION__);
            octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
            // 再创建一个局部地图管理器，单独管理局部地图对象
            // 如果后期不需要很大的修改，其实可以把局部地图的逻辑放在全局地图中，这样才符合模板的使用理念
            local_octomap_generator = new OctomapGenerator<PCLSemanticsMax, LocalSemanticsOctreeMax>();
        }
        service_ = nh_.advertiseService("toggle_use_semantic_color", &OctomapGeneratorNode::toggleUseSemanticColor, this);
    } else {
        ROS_INFO("[%s]: [%s] Color octomap generator", kNodeName.c_str(), __FUNCTION__);
        // 因为没有为 ColorOcTree 加上局部地图的功能，为了能够使模板编译通过，所以注释掉
        //octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();
    }

    reset();

    // 发布全局地图和局部地图
    fullmap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_generator_full", 1, true);
    local_map_pub = nh_.advertise<octomap_msgs::Octomap>("octomap_generator_local", 1, true);

    bool latchedTopics = true;
    //grid_map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("octomap_generator_projected_map", 5, latchedTopics);

    // 订阅小车的状态，获取线速度和角速度
    std::string scout_status;
    nh_.param<std::string>("scout_status", scout_status, "/scout_status");
    sub_scout_status = nh_.subscribe(scout_status, 1, &OctomapGeneratorNode::ScoutStatusCallback, this);

    // 根据八叉树的类型选择订阅哪种类型的语义点云
    std::string pointcloud_topic;
    if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN)
        pointcloud_topic = bayes_pointcloud_topic_;
    else
        pointcloud_topic = max_pointcloud_topic_;

    pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, pointcloud_topic, 5);
    tf_pointcloud_sub_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_sub_, tf_listener_, world_frame_id_, 5);
    tf_pointcloud_sub_->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
}

OctomapGeneratorNode::~OctomapGeneratorNode() { }
/// Clear octomap and reset values to paramters from parameter server
void OctomapGeneratorNode::reset()
{
    nh_.getParam("/octomap/max_pointcloud_topic", max_pointcloud_topic_);
    nh_.getParam("/octomap/bayes_pointcloud_topic", bayes_pointcloud_topic_);
    nh_.getParam("/octomap/world_frame_id", world_frame_id_);
    nh_.getParam("/octomap/resolution", resolution_);
    nh_.getParam("/octomap/max_range", max_range_);
    nh_.getParam("/octomap/raycast_range", raycast_range_);
    nh_.getParam("/octomap/clamping_thres_min", clamping_thres_min_);
    nh_.getParam("/octomap/clamping_thres_max", clamping_thres_max_);
    nh_.getParam("/octomap/occupancy_thres", occupancy_thres_);
    nh_.getParam("/octomap/prob_hit", prob_hit_);
    nh_.getParam("/octomap/prob_miss", prob_miss_);
    nh_.getParam("/octomap/tree_type", tree_type_);

    nh_.getParam("/octomap/outrem_radius", m_outrem_radius);
    nh_.getParam("/octomap/outrem_neighbors", m_outrem_neighbors);

    octomap_generator_->setClampingThresMin(clamping_thres_min_);
    octomap_generator_->setClampingThresMax(clamping_thres_max_);
    octomap_generator_->setResolution(resolution_);
    octomap_generator_->setOccupancyThres(occupancy_thres_);
    octomap_generator_->setProbHit(prob_hit_);
    octomap_generator_->setProbMiss(prob_miss_);
    octomap_generator_->setRayCastRange(raycast_range_);
    octomap_generator_->setMaxRange(max_range_);

    // 初始化局部地图属性
    local_octomap_generator->setClampingThresMin(clamping_thres_min_);
    local_octomap_generator->setClampingThresMax(clamping_thres_max_);
    local_octomap_generator->setResolution(resolution_);
    local_octomap_generator->setOccupancyThres(occupancy_thres_);
    local_octomap_generator->setProbHit(prob_hit_);
    local_octomap_generator->setProbMiss(prob_miss_);
    local_octomap_generator->setRayCastRange(raycast_range_);
    local_octomap_generator->setMaxRange(max_range_);
}

bool OctomapGeneratorNode::toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
#if 0    
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());

    if (octomap_generator_->isUseSemanticColor())
        ROS_INFO("Full Octomap using semantic color");
    else
        ROS_INFO("Full Octomap using rgb color");

    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing Full OctoMap");

    local_octomap_generator->setUseSemanticColor(!local_octomap_generator->isUseSemanticColor());

    if (local_octomap_generator->isUseSemanticColor())
        ROS_INFO("Local Octomap using semantic color");
    else
        ROS_INFO("Local Octomap using rgb color");

    if (octomap_msgs::fullMapToMsg(*local_octomap_generator->getOctree(), local_map_msg))
        local_map_pub.publish(local_map_msg);
    else
        ROS_ERROR("Error serializing Local OctoMap");
#endif 

    std::string save_path;
    nh_.getParam("/octomap/save_path", save_path);
    this->save(save_path.c_str());
    ROS_INFO("OctoMap saved.");

    return true;
}

void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // 对一帧融合后的点云进行半径滤波
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;

    // 设置输入点云，这里要传递指针，所以做个 share_ptr 的拷贝
    outrem.setInputCloud(cloud);

    // 设置滤波半径，launch 中配置
    outrem.setRadiusSearch(m_outrem_radius);

    // 设置近邻数量，launch 中配置
    outrem.setMinNeighborsInRadius(m_outrem_neighbors);

    // 执行半径滤波
    outrem.filter(*cloud);

    // Get tf transform
    tf::StampedTransform sensorToWorldTf;
    try {
        tf_listener_.lookupTransform(world_frame_id_, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensorToWorldTf);
    } catch (tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    // Transform coordinate
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

#if 0
    // 对 cloud 过滤地面点云
    // pc2 -> pc
    PCLSemanticsMax pc_cloud;
    pcl::fromPCLPointCloud2(*cloud, pc_cloud);

    // 把点云帧转到 base_link 坐标系？

    // segmention
    PCLSemanticsMax pc_ground_cloud;
    PCLSemanticsMax pc_no_ground_cloud;
    FilterGroundPlane(pc_cloud, pc_ground_cloud, pc_no_ground_cloud);

    // pc -> pc2
    pcl::toPCLPointCloud2(pc_no_ground_cloud, *cloud);
#endif 

    // 小车静止不动也需要插入地图
    octomap_generator_->insertPointCloud(cloud, sensorToWorld);
    local_octomap_generator->insertPointCloud(cloud, sensorToWorld);

    // Publish octomap
    map_msg_.header.frame_id = world_frame_id_;
    map_msg_.header.stamp = cloud_msg->header.stamp;
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing Full OctoMap");

#if TEST_VEL
    // 在自己的机器上使用固定的速度测试，在小车上使用真实的速度
    float v = 1.0;
    unsigned int time_thres = (1 / v) + 5;
    local_octomap_generator->UpdateLocalMap(time_thres);
#else
    float EPSINON = 0.000001;
    // 线速度不为 0 时才更新局部地图，根据小车速度和传感器最大融合范围更新局部地图消失速度
    if (linear_velocity >= EPSINON) {
        // 时间间隔的更新还需要优化！
        unsigned int time_thres = (max_range_ / linear_velocity);
        local_octomap_generator->UpdateLocalMap(time_thres);
    }
#endif

    // 发布局部地图消息
    local_map_msg.header.frame_id = world_frame_id_;
    local_map_msg.header.stamp = cloud_msg->header.stamp;
    if (octomap_msgs::fullMapToMsg(*local_octomap_generator->getOctree(), local_map_msg))
        local_map_pub.publish(local_map_msg);
    else
        ROS_ERROR("Error serializing Local OctoMap");
}

// 还是不能使用，编译不能通过！
void OctomapGeneratorNode::FilterGroundPlane(const PCLSemanticsMax& pc, PCLSemanticsMax& ground, PCLSemanticsMax& nonground) const
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // undefined reference to `pcl::PCLBase<PointXYZRGBSemanticsMax>::setIndices(boost::shared_ptr<pcl::PointIndices const> const&)
    
    // 不能分割自己的点云类型
    //pcl::SACSegmentation<PointXYZRGBSemanticsMax> seg;
    pcl::SACSegmentation<pcl::PointXYZ> seg2;

    // pc2 类型也不行
    //pcl::SACSegmentation<pcl::PCLPointCloud2> seg3;

#if 0
    ground.header = pc.header;
    nonground.header = pc.header;

    // plane detection for ground plane removal
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointXYZRGBSemanticsMax> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.01);
    //seg.setAxis(Eigen::Vector3f(0, 0, 1));
    //seg.setEpsAngle(m_groundFilterAngle);

    PCLSemanticsMax cloud_filtered(pc);
    pcl::ExtractIndices<PointXYZRGBSemanticsMax> extract;

    while (cloud_filtered.size() > 10) {
        seg.setInputCloud(cloud_filtered.makeShared());
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_INFO("FilterGroundPlane: PCL segmentation did not find any plane.");
            break;
        }

        //extract.setInputCloud(cloud_filtered.makeShared());
        //extract.setIndices(inliers);

        extract.setNegative(false);
        extract.filter(ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size()) {
            extract.setNegative(true);
            PCLSemanticsMax cloud_out;
            // 滤出非地面点云
            extract.filter(cloud_out);
            nonground += cloud_out;

            // 执行下一次地面分割
            cloud_filtered = cloud_out;
        }
    }
#endif
}

void OctomapGeneratorNode::ScoutStatusCallback(const scout_msgs::ScoutStatus::ConstPtr& scout_status)
{
    // 获取小车的线速度和角速度
    linear_velocity = scout_status->linear_velocity;
    angular_velocity = scout_status->angular_velocity;
}

bool OctomapGeneratorNode::save(const char* filename) const
{
    octomap_generator_->save(filename);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomap_generator");
    ros::NodeHandle nh;
    OctomapGeneratorNode octomapGeneratorNode(nh);
    ros::spin();

    // 暂时不需要保存地图，这个 save 暂时还有 bug！
    //std::string save_path;
    //nh.getParam("/octomap/save_path", save_path);
    //octomapGeneratorNode.save(save_path.c_str());
    //ROS_INFO("OctoMap saved.");
    return 0;
}
