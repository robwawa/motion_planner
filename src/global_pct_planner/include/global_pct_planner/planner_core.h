#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "global_pct_planner/tomogram.h"

namespace global_pct_planner {

struct DynamicSnapshot;

struct SnapResult {
  bool found{false};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  int layer{-1};
  float distance{0.0f};
};

struct PlanOutput {
  bool found{false};
  std::vector<PointXYZ> path;
  std::vector<int> layers;
};

class PlannerCore {
 public:
  explicit PlannerCore(float traversability_threshold = 49.0f);
  ~PlannerCore();

  void load(const TomogramData& map, float max_heading_rate, bool use_quintic);
  SnapResult snapToTraversable(const PointXYZ& position, float reference_height,
                               int radius_cells) const;
  bool plan(const SnapResult& start, const SnapResult& goal,
            float reference_height, bool optimize, PlanOutput& output);

  void setDynamicSnapshot(std::shared_ptr<const DynamicSnapshot> snapshot);
  void clearDynamicSnapshot();
  const TomogramData& map() const { return map_; }
  float threshold() const { return threshold_; }

 private:
  bool posToGrid(float x, float y, int& row, int& col) const;
  void gridPathToWorld(const Eigen::MatrixXd& grid_path,
                       float reference_height, PlanOutput& output) const;
  void optimizedPathToWorld(float reference_height, PlanOutput& output) const;

  float threshold_{49.0f};
  TomogramData map_;
  bool loaded_{false};
  bool use_quintic_{true};
  float max_heading_rate_{10.0f};
  std::shared_ptr<const DynamicSnapshot> dynamic_snapshot_;

  // Kept opaque in the header so users of the core do not need to include all
  // GPMP/GTSAM headers.
  struct NativePlanner;
  std::unique_ptr<NativePlanner> native_;
};

}  // namespace global_pct_planner
