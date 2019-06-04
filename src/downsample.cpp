#include <euclidean_clustering/downsample.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"


perception::Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

// Add this to your #includes


// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void perception::Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Downsampler got %ld points", cloud->size());

  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.05);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);

  ROS_INFO("Downsampled to %ld point", downsampled_cloud->size());
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  pub_.publish(msg_out);
}