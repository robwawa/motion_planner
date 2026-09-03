#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "dynamic_perception_3d/dynamic_clusterer.hpp"
#include "dynamic_perception_3d/obstacle_manager.hpp"
#include "dynamic_perception_3d/static_map_filter.hpp"

namespace dynamic_perception_3d {

class DynamicPerceptionNode {
 public:
 DynamicPerceptionNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  struct PreprocessedScans {
    CloudT::Ptr marking{new CloudT};
    // Full valid sensor coverage used only for visibility and free-space
    // evidence.  It is deliberately not restricted to the base local window.
    CloudT::Ptr visibility{new CloudT};
  };

  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& message);
  PreprocessedScans PreprocessScans(CloudT::ConstPtr map_cloud,
                                    const SensorPose& pose) const;
  void PublishCloud(const ros::Publisher& publisher, CloudT::ConstPtr cloud,
                    const ros::Time& stamp) const;
  void PublishHealth(bool healthy) const;
  void PublishState(const ros::Time& stamp);
  void PublishUnhealthy(const ros::Time& stamp, const std::string& reason);
  void PublishMarkers(const ros::Time& stamp) const;
  void PublishWindowMarkers(const ros::Time& stamp) const;
  void ValidateParameters() const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  SensorPose last_sensor_pose_;
  bool has_last_sensor_pose_ = false;

  std::string static_pcd_file_;
  std::string map_frame_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::string lidar_topic_;
  double tf_timeout_ = 0.10;
  double voxel_leaf_size_ = 0.08;
  double visibility_voxel_leaf_size_ = 0.05;
  double perception_window_size_ = 5.0;
  double min_range_ = 0.30;
  double max_range_ = 10.0;
  double min_z_ = -2.0;
  double max_z_ = 3.0;
  double marking_minimum_height_ = 0.05;
  double marking_height_ = 2.0;
  double history_window_min_height_ = 0.05;
  double history_window_max_height_ = 2.0;
  double self_filter_radius_ = 0.30;
  double self_filter_z_min_ = -0.40;
  double self_filter_z_max_ = 0.50;
  double horizontal_fov_min_deg_ = -180.0;
  double horizontal_fov_max_deg_ = 180.0;
  double vertical_fov_bottom_deg_ = -7.0;
  double vertical_fov_top_deg_ = 55.0;
  double static_cluster_ratio_ = 0.70;
  int min_clearing_scan_points_ = 6;
  bool publish_debug_ = true;

  StaticMapFilter static_filter_;
  DynamicClusterer clusterer_;
  std::unique_ptr<ObstacleManager> obstacle_manager_;

  ros::Subscriber cloud_subscriber_;
  ros::Publisher dynamic_cloud_publisher_;
  ros::Publisher scan_healthy_publisher_;
  ros::Publisher current_lidar_publisher_;
  ros::Publisher static_map_publisher_;
  ros::Publisher static_matched_publisher_;
  ros::Publisher static_surface_matched_publisher_;
  ros::Publisher dynamic_candidate_publisher_;
  ros::Publisher dynamic_confirmed_publisher_;
  ros::Publisher clearing_rays_publisher_;
  ros::Publisher obstacle_markers_publisher_;
  ros::Publisher history_window_publisher_;
  ros::Publisher preprocess_window_publisher_;
};

}  // namespace dynamic_perception_3d
