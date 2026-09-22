#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "global_pct_planner/tomogram.h"

namespace global_pct_planner {

struct DynamicSnapshot {
  std::vector<uint8_t> cost;
  uint64_t version{0};
  uint8_t lethal_cost{100};
};

struct AssignedDynamicPoint {
  int layer{0};
  int row{0};
  int col{0};
  float z{0.0f};
};

class DynamicObstacleLayer {
 public:
  DynamicObstacleLayer(const TomogramData& map, float threshold,
                       uint8_t lethal_cost, float terrain_ignore_height,
                       float collision_top, float layer_height_tolerance,
                       float robot_radius, float safety_margin,
                       float inflation_radius, int max_points_per_snapshot,
                       double max_memory_mb);

  bool replaceSnapshot(const std::vector<PointXYZ>& points);
  bool clear();

  std::shared_ptr<const DynamicSnapshot> snapshot() const;
  std::vector<AssignedDynamicPoint> assignedPoints() const;
  std::vector<AssignedDynamicPoint> nonzeroCells(
      std::size_t max_points) const;

  uint32_t layers() const { return layers_; }
  uint32_t rows() const { return rows_; }
  uint32_t cols() const { return cols_; }
  float resolution() const { return resolution_; }
  float centerX() const { return center_x_; }
  float centerY() const { return center_y_; }
  uint8_t lethalCost() const { return lethal_cost_; }
  int maxPointsPerSnapshot() const { return max_points_per_snapshot_; }

  std::pair<int, int> worldToGrid(float x, float y) const;
  std::pair<float, float> gridToWorld(int row, int col) const;

 private:
  struct Offset {
    int row{0};
    int col{0};
    uint8_t cost{0};
  };

  static int roundHalfToEven(double value);
  std::vector<AssignedDynamicPoint> assignPoints(
      const std::vector<PointXYZ>& points) const;
  void rebuildCostLocked();
  bool stateEquals(const std::vector<uint8_t>& occupied,
                   const std::vector<float>& occupied_z) const;

  const TomogramData& map_;
  float threshold_{49.0f};
  uint8_t lethal_cost_{100};
  float terrain_ignore_height_{0.08f};
  float collision_top_{0.6f};
  float layer_height_tolerance_{0.25f};
  float robot_radius_{0.32f};
  float safety_margin_{0.15f};
  float inflation_radius_{0.5f};
  int max_points_per_snapshot_{200000};

  uint32_t layers_{0};
  uint32_t rows_{0};
  uint32_t cols_{0};
  float resolution_{0.1f};
  float center_x_{0.0f};
  float center_y_{0.0f};

  std::vector<Offset> offsets_;
  std::vector<uint8_t> occupied_;
  std::vector<float> occupied_z_;
  std::shared_ptr<DynamicSnapshot> snapshot_;
  std::vector<AssignedDynamicPoint> assigned_points_;
  uint64_t version_{0};
  mutable std::mutex mutex_;
};

}  // namespace global_pct_planner
