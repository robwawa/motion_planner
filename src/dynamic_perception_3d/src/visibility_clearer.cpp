#include "dynamic_perception_3d/visibility_clearer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace dynamic_perception_3d {
namespace {
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

}  // namespace

VisibilityClearer::VisibilityClearer(const Config& config) : config_(config) {
  if (config_.min_range < 0.0 || config_.max_range <= config_.min_range ||
      config_.vertical_fov_bottom_deg >= config_.vertical_fov_top_deg ||
      config_.ray_step <= 0.0 || config_.ray_search_radius <= 0.0 ||
      config_.ray_target_guard < 0.0 ||
      config_.direct_clear_search_radius <= 0.0) {
    throw std::invalid_argument("invalid visibility clearing configuration");
  }
}

bool VisibilityClearer::IsPointInsideFov(const Eigen::Vector3d& point,
                                         const SensorPose& pose) const {
  const Eigen::Vector3d sensor_point = pose.map_T_sensor.inverse() * point;
  const double horizontal_distance = std::hypot(sensor_point.x(), sensor_point.y());
  const double range = sensor_point.norm();
  if (!std::isfinite(range) || range < config_.min_range ||
      range > config_.max_range || range <= 1e-9) {
    return false;
  }
  const double horizontal = std::atan2(sensor_point.y(), sensor_point.x()) * kRadToDeg;
  const double vertical = std::atan2(sensor_point.z(), horizontal_distance) * kRadToDeg;
  return AngleInRangeDegrees(horizontal, config_.horizontal_fov_min_deg,
                             config_.horizontal_fov_max_deg) &&
         vertical >= config_.vertical_fov_bottom_deg &&
         vertical <= config_.vertical_fov_top_deg;
}

bool VisibilityClearer::IsRayOccluded(
    const Eigen::Vector3d& target, const SensorPose& pose,
    const pcl::KdTreeFLANN<PointT>& current_scan_tree) const {
  const Eigen::Vector3d origin = pose.map_T_sensor.translation();
  const Eigen::Vector3d delta = target - origin;
  const double distance = delta.norm();
  const double end_distance = distance - config_.ray_target_guard;
  if (!std::isfinite(distance) || end_distance <= config_.ray_step) return false;
  const Eigen::Vector3d direction = delta / distance;
  std::vector<int> indices;
  std::vector<float> squared_distances;
  for (double sample_distance = std::max(config_.min_range, config_.ray_step);
       sample_distance < end_distance; sample_distance += config_.ray_step) {
    const Eigen::Vector3d sample = origin + direction * sample_distance;
    PointT point;
    point.x = static_cast<float>(sample.x());
    point.y = static_cast<float>(sample.y());
    point.z = static_cast<float>(sample.z());
    indices.clear();
    squared_distances.clear();
    if (current_scan_tree.radiusSearch(point, config_.ray_search_radius, indices,
                                       squared_distances, 1) > 0) {
      return true;
    }
  }
  return false;
}

bool VisibilityClearer::IsPointOccluded(
    const Eigen::Vector3d& point, const SensorPose& pose,
    const pcl::KdTreeFLANN<PointT>& current_scan_tree) const {
  return IsRayOccluded(point, pose, current_scan_tree);
}

bool VisibilityClearer::IsPointObservedInScan(
    const Eigen::Vector3d& point,
    const pcl::KdTreeFLANN<PointT>& current_scan_tree) const {
  PointT query;
  query.x = static_cast<float>(point.x());
  query.y = static_cast<float>(point.y());
  query.z = static_cast<float>(point.z());
  std::vector<int> indices;
  std::vector<float> squared_distances;
  return current_scan_tree.radiusSearch(query, config_.direct_clear_search_radius,
                                        indices, squared_distances, 1) > 0;
}

}  // namespace dynamic_perception_3d
