#include <iostream>
#include <vector>
#include <ctime>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>




int main() {

    srand (time (NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for (std::size_t i = 0; i < cloud->points.size (); ++i) {
        cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    pcl::PointXYZ search_point;
    search_point.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    search_point.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    search_point.z = 1024.0f * rand () / (RAND_MAX + 1.0f);


    int k = 10;
    // 存储查询点近邻索引
    std::vector<int> point_inx_nkn_search(k);
    // 存储近邻点对应距离的平方
    std::vector<float> point_nkn_square_distance(k);

    std::cout << "K nearest neighbor search at (" << search_point.x
              << " " << search_point.y
              << " " << search_point.z
              << ") with K=" << k << std::endl;


    if (kdtree.nearestKSearch(search_point, k, point_inx_nkn_search, point_nkn_square_distance) > 0) {
        for (std::size_t i = 0; i < point_inx_nkn_search.size(); i++) {
            std::cout << "    "
                      << cloud->points[point_inx_nkn_search[i]].x << " "
                      << cloud->points[point_inx_nkn_search[i]].y << " "
                      << cloud->points[point_inx_nkn_search[i]].z << "(square distance: "
                      << point_nkn_square_distance[i] << ")" << std::endl;
        }
    }

    std::vector<int> point_idx_rad_search;
    std::vector<float> point_rad_square_distance;

    float rad = 256.0f * rand() / (RAND_MAX + 1.0f);

    std::cout << "Neighbors within radius search at (" << search_point.x
              << " " << search_point.y
              << " " << search_point.z
              << ") with radius=" << rad << std::endl;


    if (kdtree.radiusSearch(search_point, rad, point_idx_rad_search, point_rad_square_distance) > 0) {
        for (std::size_t i = 0; i < point_idx_rad_search.size(); i++) {
            std::cout << "    " << cloud->points[point_idx_rad_search[i]].x << " "
                      << cloud->points[point_idx_rad_search[i]].y << " "
                      << cloud->points[point_idx_rad_search[i]].z << " "
                      << "(square distance: " << point_rad_square_distance[i] << ")" << std::endl;
        }

    }

    return 0;
}
