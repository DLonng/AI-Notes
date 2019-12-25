#include <iostream>



int main() {
	pcl::PointCloud<pcl::PointXYZI>::Ptr kitti_unorganised_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr kitti_organised_cloud (new pcl::PointCloud<pcl::PointXYZI>);


	int row_size = 64;
	int col_size = 4500;
	kitti_organised_cloud->height = row_size;
    kitti_organised_cloud->width = col_size;
    kitti_organised_cloud->resize( row_size * col_size );

    for (int i = 0; i < kitti_organised_cloud->height; ++i)
    {
        for (int j = 0; j < kitti_organised_cloud->width; ++j)
        {
            kitti_organised_cloud->at( j,i ).x = NAN;
            kitti_organised_cloud->at( j,i ).y = NAN;
            kitti_organised_cloud->at( j,i ).z = NAN;
        }
    }

    float angle = 0.0;
    float distance = 0.0;
    int col_num = 0;
    int tmp_vec_point = 0;

    for (int i = 0; i < kitti_unorganised_cloud->points.size(); ++i)
    {
        for (int j = 0; j < kitti_unorganised_cloud->points[i].size(); ++j)
        {
             angle = this->computeHorResoluteAngle( in_cloud[i].at(j) ) / M_PI * 180.0f;
             col_num = static_cast<int > ( angle / velodyne_angle_res );
             out_cloud->at(col_num,i) = in_cloud[i].at(j)
         }
    }




	return 0;
}