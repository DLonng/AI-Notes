#include <iostream>
#include <sstream>
#include <stdlib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/octree_pointcloud_compression.h>


class SimpleOpenNIViewer {

public:
  SimpleOpenNIViewer() : viewer_("Point Cloud Compression Example") {}

  void CloudCb(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    if (!viewer_.wasStopped()) {
      std::stringstream compress_data;

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>());

      point_cloud_encoder_->encodePointCloud(cloud, compress_data);

      point_cloud_decoder_->decodePointCloud(compress_data, cloud_out);

      viewer_.showCloud(cloud_out);
    }
  }

  void Run() {
    bool show_statistics = true;

    pcl::io::compression_Profiles_e compress_profile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

    point_cloud_encoder_ =
      new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compress_profile, show_statistics);

    point_cloud_decoder_ = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

    pcl::Grabber* interface = new pcl::OpenNI2Grabber();

    std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    [this](const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr & cloud) {
      this->cloud_cb(cloud);
    };

    // compile file: no registerCallback function!
    boost::signals2::connection con = interface->registerCallback(f);

    interface->start();

    while (!viewer_.wasStopped()) {
      sleep(1);
    }

    interface->stop();

    delete point_cloud_encoder_;
    delete point_cloud_decoder_;
  }


  pcl::visualization::CloudViewer viewer_;

  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* point_cloud_encoder_;
  pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* point_cloud_decoder_;
};





int main() {
  SimpleOpenNIViewer v;
  v.Run();

  return 0;
}
