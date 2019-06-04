#include "euclidean_clustering/segmentation.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/PointIndices.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h";

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

PointC min_pcl;
PointC max_pcl;

namespace perception
{
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions)
{
  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
  pose->position.x = (min_pcl.x + max_pcl.x) / 2;
  pose->position.y = (min_pcl.y + max_pcl.y) / 2;
  pose->position.z = (min_pcl.z + max_pcl.z) / 2;

  pose->orientation.x = 0;
  pose->orientation.y = 0;
  pose->orientation.z = 0;
  pose->orientation.w = 1;

  dimensions->x = max_pcl.x - min_pcl.x;
  dimensions->y = max_pcl.y - min_pcl.y;
  dimensions->z = max_pcl.z - min_pcl.z;
}
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices)
{
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.1);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 1, 0, 0;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  *indices = indices_internal;

  if (indices->indices.size() == 0)
  {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void SegmentSurfaceObjects(pcl::PointCloud<PointC>::Ptr cloud, pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices)
{
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  // ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.1);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 50);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  euclid.setMinClusterSize(min_cluster_size);
  euclid.setMaxClusterSize(max_cluster_size);
  euclid.extract(*object_indices);
}

Segmenter::Segmenter(const ros::Publisher& not_surface_points_pub, const ros::Publisher& cluster_points_pub,
                     const ros::Publisher& marker_pub)
  : not_surface_points_pub_(not_surface_points_pub)
  , cluster_points_pub_(cluster_points_pub)
  , box_marker_pub_(marker_pub)
{
}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg)
{
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, plane_inliers);

  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(cloud, plane_inliers, &object_indices);

  PointCloudC::Ptr extracted_not_plane(new PointCloudC());

  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);
  extract.filter(*extracted_not_plane);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*extracted_not_plane, msg_out);
  not_surface_points_pub_.publish(msg_out);

  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices.size(); ++i)
  {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    extract.setNegative(false);
    extract.setIndices(indices);
    extract.filter(*object_cloud);
    size_t cluster_size = (indices->indices).size();
    if (cluster_size < min_size) {
        min_size = cluster_size;
    }

    if (cluster_size > max_size) {
        max_size = cluster_size;
    }
    
    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose, &object_marker.scale);
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    box_marker_pub_.publish(object_marker);
  }

  ROS_INFO("Found %ld objects. Min_size = %ld Max_size = %ld", object_indices.size(), min_size, max_size);
}
}  // namespace perception