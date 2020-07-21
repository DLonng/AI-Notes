#include <cmath>
#include <cstring> // For std::memcpy
#include <octomap_generator/octomap_generator_ros.h>
#include <octomap_msgs/conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>
#include <sstream>

#define TEST_VEL 1

OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& nh)
    : nh_(nh)
{
    nh_.getParam("/octomap/tree_type", tree_type_);
    // Initiate octree
    if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN || tree_type_ == SEMANTICS_OCTREE_MAX) {
        if (tree_type_ == SEMANTICS_OCTREE_BAYESIAN) {
            ROS_INFO("Semantic octomap generator [bayesian fusion]");
            octomap_generator_ = new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
            // Bayes 语义融合还没有完善，主要是发布频率问题
            //local_octomap_generator = new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
        } else {
            ROS_INFO("Semantic octomap generator [max fusion]");
            octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
            // 再创建一个局部地图管理器，单独管理局部地图对象
            // 如果后期不需要很大的修改，其实可以把局部地图的逻辑放在全局地图中，这样才符合模板的使用理念
            local_octomap_generator = new OctomapGenerator<PCLSemanticsMax, LocalSemanticsOctreeMax>();
        }
        service_ = nh_.advertiseService("toggle_use_semantic_color", &OctomapGeneratorNode::toggleUseSemanticColor, this);
    } else {
        ROS_INFO("Color octomap generator");
        // 因为没有为 ColorOcTree 加上局部地图的功能，为了能够使模板编译通过，所以注释掉
        //octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();
    }

    reset();

    // 发布全局地图和局部地图
    fullmap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);
    local_map_pub = nh_.advertise<octomap_msgs::Octomap>("octomap_local", 1, true);

    // 订阅小车的状态，获取线速度和角速度
    std::string scout_status;
    nh_.param<std::string>("scout_status", scout_status, "/scout_status");
    sub_scout_status = nh_.subscribe(scout_status, 1, &OctomapGeneratorNode::ScoutStatusCallback, this);

    pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, pointcloud_topic_, 5);
    tf_pointcloud_sub_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pointcloud_sub_, tf_listener_, world_frame_id_, 5);
    tf_pointcloud_sub_->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
}

OctomapGeneratorNode::~OctomapGeneratorNode() { }
/// Clear octomap and reset values to paramters from parameter server
void OctomapGeneratorNode::reset()
{
    nh_.getParam("/octomap/pointcloud_topic", pointcloud_topic_);
    nh_.getParam("/octomap/world_frame_id", world_frame_id_);
    nh_.getParam("/octomap/resolution", resolution_);
    nh_.getParam("/octomap/max_range", max_range_);
    nh_.getParam("/octomap/raycast_range", raycast_range_);
    nh_.getParam("/octomap/clamping_thres_min", clamping_thres_min_);
    nh_.getParam("/octomap/clamping_thres_max", clamping_thres_max_);
    nh_.getParam("/octomap/occupancy_thres", occupancy_thres_);
    nh_.getParam("/octomap/prob_hit", prob_hit_);
    nh_.getParam("/octomap/prob_miss", prob_miss_);
    nh_.getParam("/tree_type", tree_type_);

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
    octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());
    if (octomap_generator_->isUseSemanticColor())
        ROS_INFO("Using semantic color");
    else
        ROS_INFO("Using rgb color");
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing OctoMap");
    return true;
}

void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // Voxel filter to down sample the point cloud
    // Create the filtering object
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*cloud_msg, *cloud);
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

    octomap_generator_->insertPointCloud(cloud, sensorToWorld);
    local_octomap_generator->insertPointCloud(cloud, sensorToWorld);

    // Publish octomap
    map_msg_.header.frame_id = world_frame_id_;
    map_msg_.header.stamp = cloud_msg->header.stamp;
    if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
        fullmap_pub_.publish(map_msg_);
    else
        ROS_ERROR("Error serializing Full OctoMap");

    // 在自己的机器上使用固定的速度测试，在小车上使用真实的速度
#if TEST_VEL
    // 更新局部地图并发布主题
    float v = 1.0;
    unsigned int time_thres = (1 / v) + 5;
    local_octomap_generator->UpdateLocalMap(time_thres);
#else
    float EPSINON = 0.000001;
    // 线速度不为 0，根据速度快慢更新局部地图消失速度
    if (linear_velocity >= EPSINON) {
        // 5 是当速度很快时，预留的局部地图显示范围
        unsigned int time_thres = (1 / linear_velocity) + 5;
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
