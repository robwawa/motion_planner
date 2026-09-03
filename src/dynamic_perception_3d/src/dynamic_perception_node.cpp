#include "dynamic_perception_3d/dynamic_perception_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <XmlRpcValue.h>
#include <geometry_msgs/Point.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_eigen/tf2_eigen.h>

namespace dynamic_perception_3d {
namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

geometry_msgs::Point ToPoint(const Eigen::Vector3d& point) {
  geometry_msgs::Point result;
  result.x = point.x();
  result.y = point.y();
  result.z = point.z();
  return result;
}

void AddBoxEdges(const Eigen::Affine3d& map_T_box, double half_x,
                 double half_y, double min_z, double max_z,
                 visualization_msgs::Marker* marker) {
  if (!marker) return;
  const Eigen::Vector3d corners[] = {
      {-half_x, -half_y, min_z}, {half_x, -half_y, min_z},
      {half_x, half_y, min_z},   {-half_x, half_y, min_z},
      {-half_x, -half_y, max_z}, {half_x, -half_y, max_z},
      {half_x, half_y, max_z},   {-half_x, half_y, max_z}};
  const int edges[][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                          {4, 5}, {5, 6}, {6, 7}, {7, 4},
                          {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& edge : edges) {
    marker->points.push_back(ToPoint(map_T_box * corners[edge[0]]));
    marker->points.push_back(ToPoint(map_T_box * corners[edge[1]]));
  }
}

Eigen::Vector3d PointOnSensorEnvelope(double range, double horizontal_deg,
                                      double vertical_deg) {
  const double horizontal = horizontal_deg / kRadToDeg;
  const double vertical = vertical_deg / kRadToDeg;
  const double horizontal_projection = range * std::cos(vertical);
  return {horizontal_projection * std::cos(horizontal),
          horizontal_projection * std::sin(horizontal), range * std::sin(vertical)};
}

void AddSensorEnvelopeEdges(const Eigen::Affine3d& map_T_sensor,
                            double min_range, double max_range,
                            double horizontal_min_deg, double horizontal_max_deg,
                            double vertical_min_deg, double vertical_max_deg,
                            visualization_msgs::Marker* marker) {
  if (!marker) return;
  double horizontal_span = horizontal_max_deg - horizontal_min_deg;
  if (horizontal_span < 0.0) horizontal_span += 360.0;
  const bool is_full_horizontal_fov = horizontal_span >= 359.999;
  const int segments = std::max(1, static_cast<int>(std::ceil(horizontal_span / 5.0)));
  const auto add_edge = [&](const Eigen::Vector3d& first,
                            const Eigen::Vector3d& second) {
    marker->points.push_back(ToPoint(map_T_sensor * first));
    marker->points.push_back(ToPoint(map_T_sensor * second));
  };

  for (int index = 1; index <= segments; ++index) {
    const double previous_horizontal =
        horizontal_min_deg + horizontal_span * (index - 1) / segments;
    const double horizontal =
        horizontal_min_deg + horizontal_span * index / segments;
    for (double range : {min_range, max_range}) {
      add_edge(PointOnSensorEnvelope(range, previous_horizontal, vertical_min_deg),
               PointOnSensorEnvelope(range, horizontal, vertical_min_deg));
      add_edge(PointOnSensorEnvelope(range, previous_horizontal, vertical_max_deg),
               PointOnSensorEnvelope(range, horizontal, vertical_max_deg));
    }
  }

  const std::vector<double> boundary_horizontals =
      is_full_horizontal_fov
          ? std::vector<double>{horizontal_min_deg}
          : std::vector<double>{horizontal_min_deg, horizontal_min_deg + horizontal_span};
  for (double horizontal : boundary_horizontals) {
    add_edge(PointOnSensorEnvelope(min_range, horizontal, vertical_min_deg),
             PointOnSensorEnvelope(max_range, horizontal, vertical_min_deg));
    add_edge(PointOnSensorEnvelope(min_range, horizontal, vertical_max_deg),
             PointOnSensorEnvelope(max_range, horizontal, vertical_max_deg));
    for (double range : {min_range, max_range}) {
      add_edge(PointOnSensorEnvelope(range, horizontal, vertical_min_deg),
               PointOnSensorEnvelope(range, horizontal, vertical_max_deg));
    }
  }
}

visualization_msgs::Marker MakeWindowMarker(
    const std::string& frame_id, const ros::Time& stamp,
    const std::string& name, const std_msgs::ColorRGBA& color) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = stamp;
  marker.ns = name;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.03;
  marker.color = color;
  marker.lifetime = ros::Duration(0.0);
  return marker;
}

Eigen::Affine3d PoseFromXyzRpy(const std::vector<double>& pose) {
  if (pose.size() != 6u ||
      !std::all_of(pose.begin(), pose.end(),
                   [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument(
        "static_map_pose must be [x, y, z, roll, pitch, yaw] with finite values");
  }
  return Eigen::Translation3d(pose[0], pose[1], pose[2]) *
         Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());
}

std::vector<double> ReadStaticMapPose(const ros::NodeHandle& private_nh) {
  XmlRpc::XmlRpcValue value;
  if (!private_nh.getParam("static_map_pose", value))
    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 6) {
    throw std::invalid_argument(
        "static_map_pose must be a six-element numeric array");
  }
  std::vector<double> pose;
  pose.reserve(6);
  for (int index = 0; index < value.size(); ++index) {
    if (value[index].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
      pose.push_back(static_cast<double>(value[index]));
    } else if (value[index].getType() == XmlRpc::XmlRpcValue::TypeInt) {
      pose.push_back(static_cast<int>(value[index]));
    } else {
      throw std::invalid_argument(
          "static_map_pose elements must be numeric values");
    }
  }
  return pose;
}
}  // namespace

DynamicPerceptionNode::DynamicPerceptionNode(ros::NodeHandle nh,
                                             ros::NodeHandle private_nh)
    : nh_(std::move(nh)),
      private_nh_(std::move(private_nh)),
      tf_listener_(tf_buffer_) {
  private_nh_.param("static_pcd_file", static_pcd_file_, std::string());
  private_nh_.param("map_frame", map_frame_, std::string("map"));
  private_nh_.param("base_frame", base_frame_, std::string("base"));
  private_nh_.param("sensor_frame", sensor_frame_, std::string("laser_livox"));
  private_nh_.param("lidar_topic", lidar_topic_, std::string("/livox/Pointcloud2"));
  private_nh_.param("tf_timeout", tf_timeout_, tf_timeout_);
  private_nh_.param("voxel_leaf_size", voxel_leaf_size_, voxel_leaf_size_);
  private_nh_.param("visibility_voxel_leaf_size", visibility_voxel_leaf_size_,
                    visibility_voxel_leaf_size_);
  private_nh_.param("perception_window_size", perception_window_size_,
                    perception_window_size_);
  private_nh_.param("min_range", min_range_, min_range_);
  private_nh_.param("max_range", max_range_, max_range_);
  private_nh_.param("min_z", min_z_, min_z_);
  private_nh_.param("max_z", max_z_, max_z_);
  private_nh_.param("marking_minimum_height", marking_minimum_height_,
                    marking_minimum_height_);
  private_nh_.param("marking_height", marking_height_, marking_height_);
  private_nh_.param("history_window_min_height", history_window_min_height_,
                    history_window_min_height_);
  private_nh_.param("history_window_max_height", history_window_max_height_,
                    history_window_max_height_);
  private_nh_.param("self_filter_radius", self_filter_radius_, self_filter_radius_);
  private_nh_.param("self_filter_z_min", self_filter_z_min_, self_filter_z_min_);
  private_nh_.param("self_filter_z_max", self_filter_z_max_, self_filter_z_max_);
  private_nh_.param("horizontal_fov_min", horizontal_fov_min_deg_,
                    horizontal_fov_min_deg_);
  private_nh_.param("horizontal_fov_max", horizontal_fov_max_deg_,
                    horizontal_fov_max_deg_);
  private_nh_.param("vertical_fov_bottom", vertical_fov_bottom_deg_,
                    vertical_fov_bottom_deg_);
  private_nh_.param("vertical_fov_top", vertical_fov_top_deg_,
                    vertical_fov_top_deg_);
  private_nh_.param("static_cluster_ratio", static_cluster_ratio_,
                    static_cluster_ratio_);
  private_nh_.param("min_clearing_scan_points", min_clearing_scan_points_,
                    min_clearing_scan_points_);
  private_nh_.param("publish_debug", publish_debug_, publish_debug_);

  StaticMapFilter::Config static_config;
  private_nh_.param("static_match_radius", static_config.static_match_radius,
                    static_config.static_match_radius);
  private_nh_.param("static_recheck_radius", static_config.static_recheck_radius,
                    static_config.static_recheck_radius);
  private_nh_.param("static_surface_matching_enabled",
                    static_config.static_surface_matching_enabled,
                    static_config.static_surface_matching_enabled);
  private_nh_.param("static_surface_search_radius",
                    static_config.static_surface_search_radius,
                    static_config.static_surface_search_radius);
  private_nh_.param("static_surface_normal_distance",
                    static_config.static_surface_normal_distance,
                    static_config.static_surface_normal_distance);
  private_nh_.param("static_surface_min_normal_norm",
                    static_config.static_surface_min_normal_norm,
                    static_config.static_surface_min_normal_norm);
  const std::vector<double> static_map_pose = ReadStaticMapPose(private_nh_);
  static_config.map_T_static_map = PoseFromXyzRpy(static_map_pose);
  private_nh_.param("static_ground_plane_enabled",
                    static_config.static_ground_plane_enabled,
                    static_config.static_ground_plane_enabled);
  private_nh_.param("static_ground_plane_z", static_config.static_ground_plane_z,
                    static_config.static_ground_plane_z);
  private_nh_.param("static_ground_plane_tolerance",
                    static_config.static_ground_plane_tolerance,
                    static_config.static_ground_plane_tolerance);
  static_filter_.SetConfig(static_config);

  DynamicClusterer::Config cluster_config;
  private_nh_.param("cluster_tolerance", cluster_config.cluster_tolerance,
                    cluster_config.cluster_tolerance);
  private_nh_.param("cluster_min_size", cluster_config.min_cluster_size,
                    cluster_config.min_cluster_size);
  private_nh_.param("cluster_max_size", cluster_config.max_cluster_size,
                    cluster_config.max_cluster_size);
  clusterer_.SetConfig(cluster_config);

  VisibilityClearer::Config visibility_config;
  visibility_config.min_range = min_range_;
  visibility_config.max_range = max_range_;
  visibility_config.horizontal_fov_min_deg = horizontal_fov_min_deg_;
  visibility_config.horizontal_fov_max_deg = horizontal_fov_max_deg_;
  visibility_config.vertical_fov_bottom_deg = vertical_fov_bottom_deg_;
  visibility_config.vertical_fov_top_deg = vertical_fov_top_deg_;
  private_nh_.param("ray_step", visibility_config.ray_step,
                    visibility_config.ray_step);
  private_nh_.param("ray_search_radius", visibility_config.ray_search_radius,
                    visibility_config.ray_search_radius);
  private_nh_.param("ray_target_guard", visibility_config.ray_target_guard,
                    visibility_config.ray_target_guard);
  private_nh_.param("direct_clear_search_radius",
                    visibility_config.direct_clear_search_radius,
                    visibility_config.direct_clear_search_radius);
  if (visibility_voxel_leaf_size_ > visibility_config.ray_search_radius ||
      visibility_voxel_leaf_size_ > visibility_config.direct_clear_search_radius) {
    throw std::invalid_argument(
        "visibility_voxel_leaf_size must not exceed visibility search radii");
  }

  ObstacleManager::Config manager_config;
  private_nh_.param("association_distance", manager_config.association_distance,
                    manager_config.association_distance);
  private_nh_.param("obstacle_voxel_size", manager_config.obstacle_voxel_size,
                    manager_config.obstacle_voxel_size);
  private_nh_.param("confirm_hits", manager_config.confirm_hits,
                    manager_config.confirm_hits);
  private_nh_.param("max_tentative_age", manager_config.max_tentative_age,
                    manager_config.max_tentative_age);
  manager_config.history_window_size = perception_window_size_;
  manager_config.history_min_height = history_window_min_height_;
  manager_config.history_max_height = history_window_max_height_;
  obstacle_manager_.reset(
      new ObstacleManager(manager_config, VisibilityClearer(visibility_config)));

  ValidateParameters();
  std::string load_error;
  if (!static_filter_.LoadMap(static_pcd_file_, &load_error)) {
    throw std::runtime_error(load_error);
  }

  dynamic_cloud_publisher_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/dynamic_perception/dynamic_cloud", 1);
  scan_healthy_publisher_ = nh_.advertise<std_msgs::Bool>(
      "/dynamic_perception/scan_healthy", 1, true);
  PublishHealth(false);
  if (publish_debug_) {
    current_lidar_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/dynamic_perception/current_lidar", 1);
    static_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/dynamic_perception/static_map", 1, true);
    static_matched_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/dynamic_perception/static_matched_cloud", 1);
    static_surface_matched_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/dynamic_perception/static_surface_matched_cloud", 1);
    dynamic_candidate_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/dynamic_perception/dynamic_candidate_cloud", 1);
    dynamic_confirmed_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/dynamic_perception/dynamic_confirmed_cloud", 1);
    clearing_rays_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/dynamic_perception/clearing_rays", 1);
    obstacle_markers_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/dynamic_perception/dynamic_obstacle_markers", 1);
    history_window_publisher_ = nh_.advertise<visualization_msgs::Marker>(
        "/dynamic_perception/history_window", 1);
    preprocess_window_publisher_ = nh_.advertise<visualization_msgs::Marker>(
        "/dynamic_perception/preprocess_window", 1);
    PublishCloud(static_map_publisher_, static_filter_.static_map(), ros::Time::now());
  }

  cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(
      lidar_topic_, 1, &DynamicPerceptionNode::CloudCallback, this,
      ros::TransportHints().tcpNoDelay());
  ROS_INFO("dynamic_perception_3d ready: map=%zu points, cloud=%s, frame=%s",
           static_filter_.static_map()->size(), lidar_topic_.c_str(),
           map_frame_.c_str());
  ROS_INFO("static map pose xyz/rpy=[%.3f %.3f %.3f %.3f %.3f %.3f], "
           "ground plane=%s (z=%.3f, tolerance=%.3f)",
           static_map_pose[0], static_map_pose[1], static_map_pose[2],
           static_map_pose[3], static_map_pose[4], static_map_pose[5],
           static_config.static_ground_plane_enabled ? "enabled" : "disabled",
           static_config.static_ground_plane_z,
           static_config.static_ground_plane_tolerance);
  ROS_INFO("static surface matching=%s, radius=%.3f, normal distance=%.3f; "
           "base marking height=[%.3f, %.3f]",
           static_config.static_surface_matching_enabled ? "enabled" : "disabled",
           static_config.static_surface_search_radius,
           static_config.static_surface_normal_distance, marking_minimum_height_,
           marking_height_);
}

void DynamicPerceptionNode::ValidateParameters() const {
  if (map_frame_.empty() || base_frame_.empty() || sensor_frame_.empty())
    throw std::invalid_argument("map_frame, base_frame and sensor_frame are required");
  if (tf_timeout_ < 0.0 || voxel_leaf_size_ <= 0.0 ||
      visibility_voxel_leaf_size_ <= 0.0 ||
      perception_window_size_ <= 0.0 || min_range_ < 0.0 ||
      max_range_ <= min_range_ || min_z_ >= max_z_ ||
      marking_minimum_height_ >= marking_height_ ||
      !std::isfinite(history_window_min_height_) ||
      !std::isfinite(history_window_max_height_) ||
      history_window_min_height_ >= history_window_max_height_ ||
      self_filter_radius_ < 0.0 ||
      self_filter_z_min_ >= self_filter_z_max_ || static_cluster_ratio_ < 0.0 ||
      static_cluster_ratio_ > 1.0 || min_clearing_scan_points_ <= 0) {
    throw std::invalid_argument("invalid preprocessing or static-filter parameters");
  }
}

DynamicPerceptionNode::PreprocessedScans
DynamicPerceptionNode::PreprocessScans(CloudT::ConstPtr map_cloud,
                                       const SensorPose& pose) const {
  PreprocessedScans result;
  if (!map_cloud) return result;
  result.marking->reserve(map_cloud->size());
  result.visibility->reserve(map_cloud->size());
  const Eigen::Affine3d sensor_T_map = pose.map_T_sensor.inverse();
  const Eigen::Affine3d base_T_map = pose.map_T_base.inverse();
  for (const PointT& point : map_cloud->points) {
    const Eigen::Vector3d map_point(point.x, point.y, point.z);
    const Eigen::Vector3d sensor_point = sensor_T_map * map_point;
    const Eigen::Vector3d base_point = base_T_map * map_point;
    const double range = sensor_point.norm();
    const double horizontal_distance = std::hypot(sensor_point.x(), sensor_point.y());
    const double horizontal_angle =
        std::atan2(sensor_point.y(), sensor_point.x()) * kRadToDeg;
    const double vertical_angle =
        std::atan2(sensor_point.z(), horizontal_distance) * kRadToDeg;
    const bool in_body =
        std::hypot(base_point.x(), base_point.y()) <= self_filter_radius_ &&
        base_point.z() >= self_filter_z_min_ && base_point.z() <= self_filter_z_max_;
    const bool in_base_window =
        std::abs(base_point.x()) <= perception_window_size_ &&
        std::abs(base_point.y()) <= perception_window_size_;
    const bool in_fov =
        AngleInRangeDegrees(horizontal_angle, horizontal_fov_min_deg_,
                            horizontal_fov_max_deg_) &&
        vertical_angle >= vertical_fov_bottom_deg_ &&
        vertical_angle <= vertical_fov_top_deg_;
    const bool sensor_measurement_valid =
        !in_body && in_fov && range >= min_range_ && range <= max_range_ &&
        sensor_point.z() >= min_z_ && sensor_point.z() <= max_z_;
    if (sensor_measurement_valid) result.visibility->push_back(point);
    if (sensor_measurement_valid && in_base_window &&
        base_point.z() >= marking_minimum_height_ &&
        base_point.z() <= marking_height_) {
      result.marking->push_back(point);
    }
  }
  if (!result.visibility->empty()) {
    CloudT::Ptr voxelized_visibility(new CloudT);
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(result.visibility);
    voxel_filter.setLeafSize(visibility_voxel_leaf_size_, visibility_voxel_leaf_size_,
                             visibility_voxel_leaf_size_);
    voxel_filter.filter(*voxelized_visibility);
    result.visibility = voxelized_visibility;
  }
  if (!result.marking->empty()) {
    CloudT::Ptr voxelized_marking(new CloudT);
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(result.marking);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*voxelized_marking);
    result.marking = voxelized_marking;
  }
  return result;
}

void DynamicPerceptionNode::CloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& message) {
  const ros::Time stamp = message ? message->header.stamp : ros::Time::now();
  if (!message || message->header.frame_id.empty()) {
    ROS_WARN_THROTTLE(2.0, "dynamic perception rejected cloud with empty frame_id");
    PublishUnhealthy(stamp, "empty cloud frame_id");
    return;
  }
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const sensor_msgs::PointField& field : message->fields) {
    has_x = has_x || field.name == "x";
    has_y = has_y || field.name == "y";
    has_z = has_z || field.name == "z";
  }
  if (!has_x || !has_y || !has_z) {
    ROS_WARN_THROTTLE(2.0, "dynamic perception requires x/y/z PointCloud2 fields");
    PublishUnhealthy(stamp, "cloud has no x/y/z fields");
    return;
  }
  try {
    const ros::Duration timeout(tf_timeout_);
    Eigen::Affine3d map_T_cloud = Eigen::Affine3d::Identity();
    if (message->header.frame_id != map_frame_) {
      map_T_cloud = tf2::transformToEigen(tf_buffer_.lookupTransform(
          map_frame_, message->header.frame_id, message->header.stamp, timeout));
    }
    SensorPose pose;
    pose.map_T_sensor = tf2::transformToEigen(tf_buffer_.lookupTransform(
        map_frame_, sensor_frame_, message->header.stamp, timeout));
    pose.map_T_base = tf2::transformToEigen(tf_buffer_.lookupTransform(
        map_frame_, base_frame_, message->header.stamp, timeout));
    last_sensor_pose_ = pose;
    has_last_sensor_pose_ = true;

    CloudT::Ptr raw(new CloudT);
    pcl::fromROSMsg(*message, *raw);
    std::vector<int> valid_indices;
    raw->is_dense = false;
    pcl::removeNaNFromPointCloud(*raw, *raw, valid_indices);
    CloudT::Ptr map_cloud(new CloudT);
    pcl::transformPointCloud(*raw, *map_cloud, map_T_cloud.matrix().cast<float>());
    const PreprocessedScans scans = PreprocessScans(map_cloud, pose);
    CloudT::Ptr current_scan = scans.marking;
    CloudT::Ptr visibility_scan = scans.visibility;
    if (visibility_scan->size() <
        static_cast<std::size_t>(min_clearing_scan_points_)) {
      PublishUnhealthy(
          message->header.stamp,
          "preprocessed scan has insufficient clearing evidence");
      return;
    }
    const StaticMapFilter::Result subtraction = static_filter_.Filter(current_scan);
    const std::vector<CloudT::Ptr> extracted =
        clusterer_.Extract(subtraction.dynamic_candidates);
    std::vector<CloudT::Ptr> accepted;
    accepted.reserve(extracted.size());
    for (const CloudT::Ptr& cluster : extracted) {
      if (static_filter_.ComputeStaticRatio(cluster) <= static_cluster_ratio_)
        accepted.push_back(cluster);
    }
    CloudT::Ptr accepted_observations(new CloudT);
    for (const CloudT::Ptr& cluster : accepted) {
      if (cluster) *accepted_observations += *cluster;
    }
    obstacle_manager_->Update(accepted, visibility_scan, accepted_observations, pose,
                              message->header.stamp.toSec());

    PublishHealth(true);
    PublishState(message->header.stamp);
    if (publish_debug_) {
      PublishCloud(current_lidar_publisher_, current_scan, message->header.stamp);
      PublishCloud(static_matched_publisher_, subtraction.static_matched,
                   message->header.stamp);
      PublishCloud(static_surface_matched_publisher_,
                   subtraction.static_surface_matched, message->header.stamp);
      PublishCloud(dynamic_candidate_publisher_, subtraction.dynamic_candidates,
                   message->header.stamp);
      PublishMarkers(message->header.stamp);
    }
    ROS_DEBUG_THROTTLE(1.0,
                       "dynamic perception: scan=%zu static=%zu surface=%zu candidates=%zu clusters=%zu",
                       current_scan->size(), subtraction.static_matched->size(),
                       subtraction.static_surface_matched->size(),
                       subtraction.dynamic_candidates->size(), accepted.size());
  } catch (const tf2::TransformException& error) {
    ROS_WARN_THROTTLE(2.0, "dynamic perception TF unavailable: %s", error.what());
    PublishUnhealthy(stamp, "TF unavailable");
  } catch (const std::exception& error) {
    ROS_ERROR_THROTTLE(2.0, "dynamic perception rejected cloud: %s", error.what());
    PublishUnhealthy(stamp, "cloud processing failed");
  }
}

void DynamicPerceptionNode::PublishCloud(const ros::Publisher& publisher,
                                         CloudT::ConstPtr cloud,
                                         const ros::Time& stamp) const {
  if (!publisher || !cloud) return;
  sensor_msgs::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.frame_id = map_frame_;
  message.header.stamp = stamp;
  publisher.publish(message);
}

void DynamicPerceptionNode::PublishHealth(bool healthy) const {
  if (!scan_healthy_publisher_) return;
  std_msgs::Bool message;
  message.data = healthy;
  scan_healthy_publisher_.publish(message);
}

void DynamicPerceptionNode::PublishUnhealthy(
    const ros::Time& stamp, const std::string& reason) {
  ROS_WARN_THROTTLE(2.0, "dynamic perception preserving snapshot: %s",
                    reason.c_str());
  PublishHealth(false);
  PublishState(stamp);
  if (publish_debug_) PublishMarkers(stamp);
}

void DynamicPerceptionNode::PublishState(const ros::Time& stamp) {
  CloudT::Ptr output = obstacle_manager_->GetOutputCloud();
  PublishCloud(dynamic_cloud_publisher_, output, stamp);
  if (publish_debug_)
    PublishCloud(dynamic_confirmed_publisher_, output, stamp);
}

void DynamicPerceptionNode::PublishMarkers(const ros::Time& stamp) const {
  visualization_msgs::MarkerArray obstacle_array;
  visualization_msgs::Marker clear;
  clear.header.frame_id = map_frame_;
  clear.header.stamp = stamp;
  clear.action = visualization_msgs::Marker::DELETEALL;
  obstacle_array.markers.push_back(clear);
  for (const DynamicObstacle& obstacle : obstacle_manager_->GetObstacles()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = stamp;
    marker.ns = "dynamic_obstacles";
    marker.id = static_cast<int>(obstacle.id & 0x7fffffff);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = ToPoint(obstacle.centroid);
    marker.pose.orientation.w = 1.0;
    const Eigen::Vector3d size = obstacle.max_bound - obstacle.min_bound;
    marker.scale.x = std::max(0.05, size.x());
    marker.scale.y = std::max(0.05, size.y());
    marker.scale.z = std::max(0.05, size.z());
    marker.color.a = 0.55;
    if (obstacle.state == ObstacleState::TENTATIVE) {
      marker.color.r = 1.0;
      marker.color.g = 0.8;
    } else if (obstacle.state == ObstacleState::OCCLUDED) {
      marker.color.r = 0.8;
      marker.color.b = 1.0;
    } else {
      marker.color.r = 1.0;
    }
    obstacle_array.markers.push_back(marker);
  }
  obstacle_markers_publisher_.publish(obstacle_array);

  visualization_msgs::MarkerArray ray_array;
  visualization_msgs::Marker rays;
  rays.header.frame_id = map_frame_;
  rays.header.stamp = stamp;
  rays.ns = "visibility_rays";
  rays.id = 0;
  rays.type = visualization_msgs::Marker::LINE_LIST;
  rays.action = visualization_msgs::Marker::ADD;
  rays.pose.orientation.w = 1.0;
  rays.scale.x = 0.02;
  for (const VisibilityRay& ray : obstacle_manager_->GetLastVisibilityRays()) {
    rays.points.push_back(ToPoint(ray.start));
    rays.points.push_back(ToPoint(ray.end));
    std_msgs::ColorRGBA color;
    color.a = 0.8;
    if (ray.evidence == ClearEvidence::OCCLUDED) {
      color.r = 1.0;
    } else if (ray.evidence == ClearEvidence::CLEARED) {
      color.g = 1.0;
    } else if (ray.evidence == ClearEvidence::OBSERVED) {
      color.b = 1.0;
    } else {
      color.r = color.g = color.b = 0.55;
    }
    rays.colors.push_back(color);
    rays.colors.push_back(color);
  }
  ray_array.markers.push_back(rays);
  clearing_rays_publisher_.publish(ray_array);
  PublishWindowMarkers(stamp);
}

void DynamicPerceptionNode::PublishWindowMarkers(
    const ros::Time& stamp) const {
  if (!has_last_sensor_pose_) return;

  std_msgs::ColorRGBA history_color;
  history_color.r = 0.1F;
  history_color.g = 1.0F;
  history_color.b = 0.9F;
  history_color.a = 0.85F;
  visualization_msgs::Marker history = MakeWindowMarker(
      map_frame_, stamp, "dynamic_perception_history_window", history_color);
  AddBoxEdges(last_sensor_pose_.map_T_base, perception_window_size_,
              perception_window_size_, history_window_min_height_,
              history_window_max_height_, &history);
  history_window_publisher_.publish(history);

  std_msgs::ColorRGBA preprocess_color;
  preprocess_color.r = 1.0F;
  preprocess_color.g = 0.8F;
  preprocess_color.b = 0.1F;
  preprocess_color.a = 0.70F;
  visualization_msgs::Marker preprocess = MakeWindowMarker(
      map_frame_, stamp, "dynamic_perception_preprocess_window",
      preprocess_color);
  AddSensorEnvelopeEdges(last_sensor_pose_.map_T_sensor, min_range_, max_range_,
                         horizontal_fov_min_deg_, horizontal_fov_max_deg_,
                         vertical_fov_bottom_deg_, vertical_fov_top_deg_,
                         &preprocess);
  preprocess_window_publisher_.publish(preprocess);
}

}  // namespace dynamic_perception_3d
