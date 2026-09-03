#pragma once

#include <pcl/kdtree/kdtree_flann.h>

#include "dynamic_perception_3d/common_types.hpp"

namespace dynamic_perception_3d {

class VisibilityClearer {
 public:
  struct Config {
    double min_range = 0.30;
    double max_range = 10.0;
    double horizontal_fov_min_deg = -180.0;
    double horizontal_fov_max_deg = 180.0;
    double vertical_fov_bottom_deg = -7.0;
    double vertical_fov_top_deg = 55.0;
    double ray_step = 0.05;
    double ray_search_radius = 0.10;
    double ray_target_guard = 0.20;
    double direct_clear_search_radius = 0.10;
  };

  VisibilityClearer() = default;
  explicit VisibilityClearer(const Config& config);

  bool IsPointInsideFov(const Eigen::Vector3d& point,
                        const SensorPose& pose) const;
  bool IsPointOccluded(const Eigen::Vector3d& point,
                       const SensorPose& pose,
                       const pcl::KdTreeFLANN<PointT>& current_scan_tree) const;
  bool IsPointObservedInScan(
      const Eigen::Vector3d& point,
      const pcl::KdTreeFLANN<PointT>& current_scan_tree) const;

 private:
  bool IsRayOccluded(const Eigen::Vector3d& target,
                     const SensorPose& pose,
                     const pcl::KdTreeFLANN<PointT>& current_scan_tree) const;
  Config config_;
};

}  // namespace dynamic_perception_3d
