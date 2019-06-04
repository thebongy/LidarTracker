#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices);
void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices);
class Segmenter {
 public:
  Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& cluster_points_pub, const ros::Publisher& marker_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher not_surface_points_pub_;
  ros::Publisher cluster_points_pub_;
  ros::Publisher box_marker_pub_;
};
}  // namespace perception