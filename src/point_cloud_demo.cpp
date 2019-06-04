#include "euclidean_clustering/crop.h"
#include "euclidean_clustering/downsample.h"
#include "euclidean_clustering/segmentation.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;
  
  ros::Publisher crop_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  ros::Publisher downsample_pub = nh.advertise<sensor_msgs::PointCloud2>("downsampled_cloud", 1, true);
  ros::Publisher not_plane_pub = nh.advertise<sensor_msgs::PointCloud2>("not_plane_cloud", 1, true);
  ros::Publisher cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("cluster_cloud", 1, true);
  
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  perception::Cropper cropper(crop_pub);
  perception::Downsampler downsampler(downsample_pub);
  perception::Segmenter segmenter(not_plane_pub, cluster_pub, marker_pub);

  // First crop input cloud
  ros::Subscriber sub = nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
  
  // TODO: Filter outliers from input pointcloud

  // Then downsample cropped cloud
  ros::Subscriber sub_crop = nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
  
  ros::Subscriber sub_downsample = nh.subscribe("downsampled_cloud", 1, &perception::Segmenter::Callback, &segmenter);
  ros::spin();
  return 0;
}