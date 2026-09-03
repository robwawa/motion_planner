#pragma once

#include <limits>
#include <mutex>
#include <vector>

#include "dynamic_perception_3d/common_types.hpp"
#include "dynamic_perception_3d/visibility_clearer.hpp"

namespace dynamic_perception_3d {

class ObstacleManager {
 public:
  struct Config {
    double association_distance = 0.50;
    double obstacle_voxel_size = 0.08;
    int confirm_hits = 2;
    // Dynamic occupancy is local evidence. Keep only voxels in this base-frame
    // XY window and the independently configured height band.
    double history_window_size = 10.0;
    double history_min_height = -std::numeric_limits<double>::infinity();
    double history_max_height = std::numeric_limits<double>::infinity();
    // TENTATIVE obstacles are never published, so a short age bound prevents
    // unconfirmed noise from accumulating without releasing planned space.
    double max_tentative_age = 2.0;
  };

  ObstacleManager(const Config& config, const VisibilityClearer& clearer);

  // visibility_scan supplies foreground-occlusion and direct target-neighborhood
  // evidence from the complete valid sensor field of view. dynamic_observations
  // remains in the internal API for call-site compatibility but is not used by
  // the DDDMR-style direct-clear policy.
  void Update(const std::vector<CloudT::Ptr>& clusters,
              CloudT::ConstPtr visibility_scan,
              CloudT::ConstPtr dynamic_observations,
              const SensorPose& sensor_pose, double now);
  void Clear();

  CloudT::Ptr GetOutputCloud() const;
  std::vector<DynamicObstacle> GetObstacles() const;
  std::vector<VisibilityRay> GetLastVisibilityRays() const;

 private:
  DynamicObstacle MakeObstacle(uint64_t id, CloudT::ConstPtr cloud,
                               double now) const;
  static void UpdateObservationGeometry(DynamicObstacle* obstacle,
                                        CloudT::ConstPtr cloud);
  static void UpdateCloudFromVoxels(DynamicObstacle* obstacle);
  ObstacleVoxelKey ToVoxelKey(const PointT& point) const;
  void MarkVoxels(DynamicObstacle* obstacle, CloudT::ConstPtr observation,
                  std::map<ObstacleVoxelKey, bool>* marked) const;
  bool PruneTentativeLocked(double now);
  void PruneOutsideHistoryWindowLocked(const SensorPose& sensor_pose);
  bool IsInsideHistoryWindow(const PointT& point,
                             const SensorPose& sensor_pose) const;

  Config config_;
  VisibilityClearer clearer_;
  mutable std::mutex mutex_;
  std::vector<DynamicObstacle> obstacles_;
  std::vector<VisibilityRay> last_visibility_rays_;
  uint64_t next_id_ = 1;
  double last_update_time_ = -std::numeric_limits<double>::infinity();
};

}  // namespace dynamic_perception_3d
