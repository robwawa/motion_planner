#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dynamic_perception_3d {

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

inline bool AngleInRangeDegrees(double angle, double minimum, double maximum) {
  if (minimum <= maximum) return angle >= minimum && angle <= maximum;
  return angle >= minimum || angle <= maximum;
}

struct SensorPose {
  Eigen::Affine3d map_T_sensor = Eigen::Affine3d::Identity();
  Eigen::Affine3d map_T_base = Eigen::Affine3d::Identity();
};

enum class ObstacleState : uint8_t {
  TENTATIVE = 0,
  CONFIRMED = 1,
  OCCLUDED = 2,
};

struct ObstacleVoxelKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  bool operator<(const ObstacleVoxelKey& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

struct ObstacleVoxel {
  PointT point;
};

struct DynamicObstacle {
  uint64_t id = 0;
  // Stable geometry is regenerated from the independently managed Mark/Clear
  // voxels below.
  CloudT::Ptr cloud{new CloudT};
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d min_bound = Eigen::Vector3d::Zero();
  Eigen::Vector3d max_bound = Eigen::Vector3d::Zero();

  std::map<ObstacleVoxelKey, ObstacleVoxel> voxels;

  // Latest accepted single-frame observation is used only for association.
  CloudT::Ptr observation_cloud{new CloudT};
  Eigen::Vector3d observation_centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d observation_min_bound = Eigen::Vector3d::Zero();
  Eigen::Vector3d observation_max_bound = Eigen::Vector3d::Zero();
  double first_seen = 0.0;
  double last_seen = 0.0;
  int hit_count = 0;
  int miss_count = 0;
  ObstacleState state = ObstacleState::TENTATIVE;
};

enum class ClearEvidence : uint8_t {
  UNKNOWN = 0,
  OCCLUDED = 1,
  OBSERVED = 2,
  CLEARED = 3,
};

struct VisibilityRay {
  Eigen::Vector3d start = Eigen::Vector3d::Zero();
  Eigen::Vector3d end = Eigen::Vector3d::Zero();
  ClearEvidence evidence = ClearEvidence::UNKNOWN;
};

}  // namespace dynamic_perception_3d
#include <cmath>
