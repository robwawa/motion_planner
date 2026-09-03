#include "dynamic_perception_3d/obstacle_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <tuple>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace dynamic_perception_3d {

ObstacleManager::ObstacleManager(const Config& config,
                                 const VisibilityClearer& clearer)
    : config_(config), clearer_(clearer) {
  if (!std::isfinite(config_.association_distance) ||
      !std::isfinite(config_.obstacle_voxel_size) ||
      !std::isfinite(config_.max_tentative_age) ||
      !std::isfinite(config_.history_window_size) ||
      config_.association_distance <= 0.0 ||
      config_.obstacle_voxel_size <= 0.0 || config_.history_window_size <= 0.0 ||
      config_.history_min_height >= config_.history_max_height ||
      config_.confirm_hits <= 0 ||
      config_.max_tentative_age <= 0.0) {
    throw std::invalid_argument("invalid obstacle lifecycle configuration");
  }
}

bool ObstacleManager::IsInsideHistoryWindow(
    const PointT& point, const SensorPose& sensor_pose) const {
  const Eigen::Vector3d base_point = sensor_pose.map_T_base.inverse() *
                                     Eigen::Vector3d(point.x, point.y, point.z);
  return std::abs(base_point.x()) <= config_.history_window_size &&
         std::abs(base_point.y()) <= config_.history_window_size &&
         base_point.z() >= config_.history_min_height &&
         base_point.z() <= config_.history_max_height;
}

void ObstacleManager::PruneOutsideHistoryWindowLocked(
    const SensorPose& sensor_pose) {
  for (DynamicObstacle& obstacle : obstacles_) {
    for (auto voxel = obstacle.voxels.begin(); voxel != obstacle.voxels.end();) {
      if (IsInsideHistoryWindow(voxel->second.point, sensor_pose)) {
        ++voxel;
      } else {
        voxel = obstacle.voxels.erase(voxel);
      }
    }
    if (!obstacle.voxels.empty()) UpdateCloudFromVoxels(&obstacle);
  }
  obstacles_.erase(
      std::remove_if(obstacles_.begin(), obstacles_.end(),
                     [](const DynamicObstacle& obstacle) {
                       return obstacle.voxels.empty();
                     }),
      obstacles_.end());
}

void ObstacleManager::UpdateObservationGeometry(DynamicObstacle* obstacle,
                                                CloudT::ConstPtr cloud) {
  obstacle->observation_cloud.reset(new CloudT(*cloud));
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  obstacle->observation_centroid = centroid.head<3>().cast<double>();
  PointT minimum;
  PointT maximum;
  pcl::getMinMax3D(*cloud, minimum, maximum);
  obstacle->observation_min_bound =
      Eigen::Vector3d(minimum.x, minimum.y, minimum.z);
  obstacle->observation_max_bound =
      Eigen::Vector3d(maximum.x, maximum.y, maximum.z);
}

void ObstacleManager::UpdateCloudFromVoxels(DynamicObstacle* obstacle) {
  obstacle->cloud.reset(new CloudT);
  obstacle->cloud->reserve(obstacle->voxels.size());
  for (const auto& entry : obstacle->voxels)
    obstacle->cloud->push_back(entry.second.point);
  if (obstacle->cloud->empty()) return;
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*obstacle->cloud, centroid);
  obstacle->centroid = centroid.head<3>().cast<double>();
  PointT minimum;
  PointT maximum;
  pcl::getMinMax3D(*obstacle->cloud, minimum, maximum);
  obstacle->min_bound = Eigen::Vector3d(minimum.x, minimum.y, minimum.z);
  obstacle->max_bound = Eigen::Vector3d(maximum.x, maximum.y, maximum.z);
}

ObstacleVoxelKey ObstacleManager::ToVoxelKey(const PointT& point) const {
  const double inverse_size = 1.0 / config_.obstacle_voxel_size;
  return {static_cast<int64_t>(std::floor(point.x * inverse_size)),
          static_cast<int64_t>(std::floor(point.y * inverse_size)),
          static_cast<int64_t>(std::floor(point.z * inverse_size))};
}

void ObstacleManager::MarkVoxels(
    DynamicObstacle* obstacle, CloudT::ConstPtr observation,
    std::map<ObstacleVoxelKey, bool>* marked) const {
  UpdateObservationGeometry(obstacle, observation);
  for (const PointT& point : observation->points) {
    const ObstacleVoxelKey key = ToVoxelKey(point);
    obstacle->voxels.emplace(key, ObstacleVoxel{point});
    // Keep the first representative point in an occupied voxel. This makes
    // the published geometry stable instead of jittering with every scan.
    if (marked) (*marked)[key] = true;
  }
  UpdateCloudFromVoxels(obstacle);
}

DynamicObstacle ObstacleManager::MakeObstacle(uint64_t id,
                                               CloudT::ConstPtr cloud,
                                               double now) const {
  DynamicObstacle obstacle;
  obstacle.id = id;
  obstacle.first_seen = now;
  obstacle.last_seen = now;
  obstacle.hit_count = 1;
  obstacle.miss_count = 0;
  obstacle.state = ObstacleState::TENTATIVE;
  MarkVoxels(&obstacle, cloud, nullptr);
  return obstacle;
}

void ObstacleManager::Update(const std::vector<CloudT::Ptr>& clusters,
                             CloudT::ConstPtr visibility_scan,
                             CloudT::ConstPtr dynamic_observations,
                             const SensorPose& sensor_pose, double now) {
  (void)dynamic_observations;
  std::lock_guard<std::mutex> lock(mutex_);
  if (now < last_update_time_) {
    obstacles_.clear();
    last_visibility_rays_.clear();
  }
  last_update_time_ = now;
  PruneTentativeLocked(now);
  PruneOutsideHistoryWindowLocked(sensor_pose);
  last_visibility_rays_.clear();

  std::vector<DynamicObstacle> detections;
  detections.reserve(clusters.size());
  for (CloudT::ConstPtr cluster : clusters) {
    if (cluster && !cluster->empty()) detections.push_back(MakeObstacle(0, cluster, now));
  }

  using Candidate = std::tuple<double, std::size_t, std::size_t>;
  std::vector<Candidate> candidates;
  for (std::size_t old_index = 0; old_index < obstacles_.size(); ++old_index) {
    for (std::size_t new_index = 0; new_index < detections.size(); ++new_index) {
      const double distance =
          (obstacles_[old_index].observation_centroid -
           detections[new_index].observation_centroid)
              .norm();
      if (distance <= config_.association_distance) {
        candidates.emplace_back(distance, old_index, new_index);
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());
  std::vector<bool> old_matched(obstacles_.size(), false);
  std::vector<bool> new_matched(detections.size(), false);
  std::vector<std::map<ObstacleVoxelKey, bool>> marked_voxels(obstacles_.size());
  for (const Candidate& candidate : candidates) {
    const std::size_t old_index = std::get<1>(candidate);
    const std::size_t new_index = std::get<2>(candidate);
    if (old_matched[old_index] || new_matched[new_index]) continue;
    DynamicObstacle& obstacle = obstacles_[old_index];
    MarkVoxels(&obstacle, detections[new_index].observation_cloud,
               &marked_voxels[old_index]);
    obstacle.last_seen = now;
    obstacle.hit_count += 1;
    obstacle.miss_count = 0;
    if (obstacle.state == ObstacleState::OCCLUDED ||
        obstacle.hit_count >= config_.confirm_hits) {
      obstacle.state = ObstacleState::CONFIRMED;
    }
    old_matched[old_index] = true;
    new_matched[new_index] = true;
  }

  pcl::KdTreeFLANN<PointT> scan_tree;
  const bool has_scan = visibility_scan && !visibility_scan->empty();
  if (has_scan) scan_tree.setInputCloud(visibility_scan);
  std::vector<bool> remove(obstacles_.size(), false);
  for (std::size_t index = 0; index < obstacles_.size(); ++index) {
    DynamicObstacle& obstacle = obstacles_[index];
    if (!old_matched[index] && obstacle.state == ObstacleState::TENTATIVE)
      obstacle.hit_count = 0;

    bool has_uncertain_voxel = false;
    for (auto voxel = obstacle.voxels.begin(); voxel != obstacle.voxels.end();) {
      if (marked_voxels[index].find(voxel->first) !=
          marked_voxels[index].end()) {
        ++voxel;
        continue;
      }

      ObstacleVoxel& evidence = voxel->second;
      const Eigen::Vector3d point(evidence.point.x, evidence.point.y,
                                  evidence.point.z);
      if (!has_scan || !clearer_.IsPointInsideFov(point, sensor_pose)) {
        if (has_scan) {
          last_visibility_rays_.push_back(
              {sensor_pose.map_T_sensor.translation(), point,
               ClearEvidence::UNKNOWN});
        }
        has_uncertain_voxel = true;
        ++voxel;
        continue;
      }

      const bool occluded = clearer_.IsPointOccluded(point, sensor_pose, scan_tree);
      if (occluded) {
        last_visibility_rays_.push_back(
            {sensor_pose.map_T_sensor.translation(), point,
             ClearEvidence::OCCLUDED});
        has_uncertain_voxel = true;
        ++voxel;
        continue;
      }

      if (clearer_.IsPointObservedInScan(point, scan_tree)) {
        last_visibility_rays_.push_back(
            {sensor_pose.map_T_sensor.translation(), point,
             ClearEvidence::OBSERVED});
        ++voxel;
        continue;
      }

      last_visibility_rays_.push_back(
          {sensor_pose.map_T_sensor.translation(), point, ClearEvidence::CLEARED});
      voxel = obstacle.voxels.erase(voxel);
    }

    if (obstacle.voxels.empty()) {
      remove[index] = true;
      continue;
    }
    UpdateCloudFromVoxels(&obstacle);
    if (obstacle.state == ObstacleState::CONFIRMED ||
        obstacle.state == ObstacleState::OCCLUDED) {
      obstacle.state = (!old_matched[index] && has_uncertain_voxel)
                           ? ObstacleState::OCCLUDED
                           : ObstacleState::CONFIRMED;
    }
  }

  std::vector<DynamicObstacle> kept;
  kept.reserve(obstacles_.size() + detections.size());
  for (std::size_t index = 0; index < obstacles_.size(); ++index) {
    if (!remove[index]) kept.push_back(std::move(obstacles_[index]));
  }
  obstacles_.swap(kept);
  for (std::size_t index = 0; index < detections.size(); ++index) {
    if (new_matched[index]) continue;
    detections[index].id = next_id_++;
    if (config_.confirm_hits <= 1) detections[index].state = ObstacleState::CONFIRMED;
    obstacles_.push_back(std::move(detections[index]));
  }
  PruneOutsideHistoryWindowLocked(sensor_pose);
}

bool ObstacleManager::PruneTentativeLocked(double now) {
  const std::size_t previous_size = obstacles_.size();
  obstacles_.erase(
      std::remove_if(obstacles_.begin(), obstacles_.end(),
                     [&](const DynamicObstacle& obstacle) {
                       return obstacle.state == ObstacleState::TENTATIVE &&
                              now - obstacle.first_seen >
                                  config_.max_tentative_age;
                     }),
      obstacles_.end());
  return obstacles_.size() != previous_size;
}

void ObstacleManager::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  obstacles_.clear();
  last_visibility_rays_.clear();
}

CloudT::Ptr ObstacleManager::GetOutputCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  CloudT::Ptr output(new CloudT);
  for (const DynamicObstacle& obstacle : obstacles_) {
    if (obstacle.state == ObstacleState::CONFIRMED ||
        obstacle.state == ObstacleState::OCCLUDED) {
      *output += *obstacle.cloud;
    }
  }
  return output;
}

std::vector<DynamicObstacle> ObstacleManager::GetObstacles() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return obstacles_;
}

std::vector<VisibilityRay> ObstacleManager::GetLastVisibilityRays() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_visibility_rays_;
}

}  // namespace dynamic_perception_3d
