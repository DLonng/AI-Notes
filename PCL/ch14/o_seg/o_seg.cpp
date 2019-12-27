#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


int main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("../table_scene_lms400.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // 创建 ransac 分割对象
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // 存储分割出的内点索引
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // 要分割的模型参数
  pcl::ModelCoefficients::Ptr coefficients (
      new pcl::ModelCoefficients);

  // 存储分割后的桌子平面
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (
      new pcl::PointCloud<pcl::PointXYZ> ());

  pcl::PCDWriter writer;
  // 设置对估计的模型参数进行优化
  seg.setOptimizeCoefficients (true);

  // 设置分割模型为平面
  seg.setModelType (pcl::SACMODEL_PLANE);

  // 设置模型的参数估计算法为 RANSAC 
  seg.setMethodType (pcl::SAC_RANSAC);

  // 设置 RANSAC 算法最大的迭代次数为 100 次
  seg.setMaxIterations (100);

  // 设置内点到模型的允许的最大距离为 2cm
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();

  int k = 0;
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // 设置输出点云
    seg.setInputCloud (cloud_filtered);

    // 执行分割，分割的内点存储在 inliers 中，估计的模型参数存储在 coefficients 中
    seg.segment (*inliers, *coefficients);

    // 如果分割的平面模型的内点数量为 0 则平面模型分割完毕
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

      // 将每个分割后的聚类写入不同的点云子集中
      for (std::vector<int>::const_iterator pit = inliers->indices.begin (); pit != inliers->indices.end (); ++pit) {
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      }

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      // 设置点云没有无效点，即无 Nan 点
      cloud_cluster->is_dense = true;

      // 输出每个聚类的点云数量并保存为 pcd 文件
      std::cout << "seg plan: " << cloud_cluster->points.size () << " data points." << std::endl;
      std::stringstream ss;
      ss << "cloud_cluster_plan_" << k++ << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    }


    // 用提取索引的方法来在原始点云中滤除已经分割的平面点云子集
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);

    // 设置输出滤除的平面子集
    extract.setNegative (false);
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // 设置输出滤除平面子集的原始剩余点云集合
    extract.setNegative (true);
    extract.filter (*cloud_f);

    // 提取剩余点云用于下一次分割
    *cloud_filtered = *cloud_f;
  }

  // 使用 kd-tree 来搜索点云
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (
      new pcl::search::KdTree<pcl::PointXYZ>);

  // 这里的输入为已经分割了桌子平面的剩余点云
  tree->setInputCloud (cloud_filtered);

  // 点云索引向量，存储每个点云聚类的索引
  // pcl::PointIndices 内部也是一个 std::vector
  std::vector<pcl::PointIndices> cluster_indices;

  // 创建欧氏分割对象
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  /*
   * 设置近邻搜索半径为 2cm
   * 注意：该参数需要调节
   * 因为半径太大容易将多个对象分为一个聚类，
   * 半径太小容易将一个对象分为多个聚类！
   */
  ec.setClusterTolerance (0.02);

  // 设置一个聚类的最少点数量为 100
  ec.setMinClusterSize (100);

  // 设置一个聚类的最大点数量为 25000
  ec.setMaxClusterSize (25000);

  // 设置点云的搜索方法为 kd-tree
  ec.setSearchMethod (tree);

  // 设置输入的滤波点云
  ec.setInputCloud (cloud_filtered);

  // 执行聚类分割，每个分割后的聚类索引保存在 cluster_indices 中
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    // 将每个分割后的聚类写入不同的点云子集中
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    // 设置点云没有无效点，即无 Nan 点
    cloud_cluster->is_dense = true;

    // 输出每个聚类的点云数量并保存为 pcd 文件
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  return (0);
}
