#include "global_pct_planner/planner_core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>

#include "ele_planner/offline_ele_planner.h"
#include "global_pct_planner/dynamic_obstacle_layer.h"

namespace global_pct_planner {
namespace {

int roundHalfToEven(float value) {
  const double floor_value = std::floor(static_cast<double>(value));
  const double fraction = static_cast<double>(value) - floor_value;
  if (fraction < 0.5) return static_cast<int>(floor_value);
  if (fraction > 0.5) return static_cast<int>(floor_value + 1.0);
  const auto lower = static_cast<long long>(floor_value);
  return static_cast<int>((lower % 2 == 0) ? lower : lower + 1);
}

// Keep the native preprocessing identical to planner_wrapper.py's
// np.nan_to_num(array, nan=...).  In particular, NaN is replaced by the
// explicit fallback, while +/-inf is replaced by the corresponding largest
// finite value of the array dtype rather than by the NaN fallback.
float legacyNanToNum(float value, float nan_replacement) {
  if (std::isnan(value)) return nan_replacement;
  if (std::isinf(value)) {
    const float max_value = std::numeric_limits<float>::max();
    return value > 0.0f ? max_value : -max_value;
  }
  return value;
}

}  // namespace

struct PlannerCore::NativePlanner {
  explicit NativePlanner(float max_heading_rate, bool use_quintic)
      : planner(max_heading_rate, use_quintic) {}
  OfflineElePlanner planner;
};

PlannerCore::PlannerCore(float traversability_threshold)
    : threshold_(traversability_threshold) {}

PlannerCore::~PlannerCore() = default;

void PlannerCore::load(const TomogramData& map, float max_heading_rate,
                       bool use_quintic) {
  if (map.layers == 0 || map.rows == 0 || map.cols == 0 ||
      map.resolution <= 0.0f || map.traversability.size() != map.cellCount() ||
      map.ground_elevation.size() != map.cellCount() ||
      map.ceiling_elevation.size() != map.cellCount() ||
      map.traversability_grad_x.size() != map.cellCount() ||
      map.traversability_grad_y.size() != map.cellCount()) {
    throw std::invalid_argument("invalid PCT map passed to planner");
  }
  map_ = map;
  max_heading_rate_ = max_heading_rate;
  use_quintic_ = use_quintic;
  native_ = std::make_unique<NativePlanner>(max_heading_rate_, use_quintic_);

  const int matrix_rows = static_cast<int>(map.layers * map.rows);
  const int matrix_cols = static_cast<int>(map.cols);
  Eigen::MatrixXd cost(matrix_rows, matrix_cols);
  Eigen::MatrixXd ground(matrix_rows, matrix_cols);
  Eigen::MatrixXd ceiling(matrix_rows, matrix_cols);
  Eigen::MatrixXd gateway = Eigen::MatrixXd::Zero(matrix_rows, matrix_cols);
  Eigen::MatrixXd grad_x(matrix_rows, matrix_cols);
  Eigen::MatrixXd grad_y(matrix_rows, matrix_cols);

  for (uint32_t layer = 0; layer < map.layers; ++layer) {
    for (uint32_t row = 0; row < map.rows; ++row) {
      const int matrix_row = static_cast<int>(layer * map.rows + row);
      for (uint32_t col = 0; col < map.cols; ++col) {
        const std::size_t source = map.index(layer, row, col);
        cost(matrix_row, col) = map.traversability[source];
        ground(matrix_row, col) = legacyNanToNum(
            map.ground_elevation[source], -100.0f);
        ceiling(matrix_row, col) = legacyNanToNum(
            map.ceiling_elevation[source], 1e6f);
        grad_x(matrix_row, col) = map.traversability_grad_y[source];
        grad_y(matrix_row, col) = -map.traversability_grad_x[source];
      }
    }
  }
  // The native GPMP implementation uses an elevation mask to enter/leave a
  // surface.  Rebuild that mask from the serialized arrays so .pctm remains
  // self-contained and no Python preprocessing is needed at runtime.
  // Match planner_wrapper.py exactly: gateway_up is assigned first and
  // gateway_dn is assigned afterwards, making -2 win when both flags are
  // present at the same [layer,row,col].
  for (uint32_t layer = 0; layer + 1 < map.layers; ++layer) {
    for (uint32_t row = 0; row < map.rows; ++row) {
      for (uint32_t col = 0; col < map.cols; ++col) {
        const std::size_t lower = map.index(layer, row, col);
        const std::size_t upper = map.index(layer + 1, row, col);
        // planner_wrapper.py computes gateway flags after
        // np.nan_to_num(elev_g_raw, nan=-100).  Therefore NaN is deliberately
        // treated as -100 here and remains eligible for the same gateway
        // comparison as in the legacy package.
        const float lower_ground = legacyNanToNum(
            map.ground_elevation[lower], -100.0f);
        const float upper_ground = legacyNanToNum(
            map.ground_elevation[upper], -100.0f);
        if (std::abs(upper_ground - lower_ground) >= 0.1f) {
          continue;
        }
        const float difference = map.traversability[upper] -
                                 map.traversability[lower];
        if (difference < -8.0f) {
          gateway(static_cast<int>(layer * map.rows + row), col) = 2.0;
        }
      }
    }
  }
  for (uint32_t layer = 0; layer + 1 < map.layers; ++layer) {
    for (uint32_t row = 0; row < map.rows; ++row) {
      for (uint32_t col = 0; col < map.cols; ++col) {
        const std::size_t lower = map.index(layer, row, col);
        const std::size_t upper = map.index(layer + 1, row, col);
        const float lower_ground = legacyNanToNum(
            map.ground_elevation[lower], -100.0f);
        const float upper_ground = legacyNanToNum(
            map.ground_elevation[upper], -100.0f);
        if (std::abs(upper_ground - lower_ground) >= 0.1f) continue;
        const float difference = map.traversability[upper] -
                                 map.traversability[lower];
        if (difference > 8.0f) {
          gateway(static_cast<int>((layer + 1) * map.rows + row), col) = -2.0;
        }
      }
    }
  }

  native_->planner.InitMap(threshold_, 15.0, map.resolution, map.layers, 0.2,
                           cost, ground, ceiling, gateway, grad_x, grad_y);
  loaded_ = true;
  if (dynamic_snapshot_) setDynamicSnapshot(dynamic_snapshot_);
}

bool PlannerCore::posToGrid(float x, float y, int& row, int& col) const {
  row = roundHalfToEven((x - map_.center_x) / map_.resolution) +
        static_cast<int>(map_.rows / 2);
  col = roundHalfToEven((y - map_.center_y) / map_.resolution) +
        static_cast<int>(map_.cols / 2);
  return row >= 0 && row < static_cast<int>(map_.rows) && col >= 0 &&
         col < static_cast<int>(map_.cols);
}

SnapResult PlannerCore::snapToTraversable(const PointXYZ& position,
                                           float reference_height,
                                           int radius_cells) const {
  SnapResult result;
  if (!loaded_ || !std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)) {
    return result;
  }
  int center_row = 0;
  int center_col = 0;
  if (!posToGrid(position.x, position.y, center_row, center_col)) return result;
  radius_cells = std::max(0, radius_cells);
  const int row_min = std::max(0, center_row - radius_cells);
  const int row_max = std::min(static_cast<int>(map_.rows) - 1,
                               center_row + radius_cells);
  const int col_min = std::max(0, center_col - radius_cells);
  const int col_max = std::min(static_cast<int>(map_.cols) - 1,
                               center_col + radius_cells);
  const uint8_t lethal = dynamic_snapshot_ ? dynamic_snapshot_->lethal_cost : 255;
  for (uint32_t layer = 0; layer < map_.layers; ++layer) {
    for (int row = row_min; row <= row_max; ++row) {
      for (int col = col_min; col <= col_max; ++col) {
        const std::size_t index = map_.index(layer, row, col);
        const float ground = map_.ground_elevation[index];
        if (!std::isfinite(ground) || map_.traversability[index] > threshold_)
          continue;
        if (dynamic_snapshot_ &&
            dynamic_snapshot_->cost[index] >= lethal) {
          continue;
        }
        const PointXYZ candidate{
            (row - static_cast<int>(map_.rows / 2)) * map_.resolution +
                map_.center_x,
            (col - static_cast<int>(map_.cols / 2)) * map_.resolution +
                map_.center_y,
            ground + reference_height};
        const float dx = candidate.x - position.x;
        const float dy = candidate.y - position.y;
        const float dz = candidate.z - position.z;
        const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (!result.found || distance < result.distance) {
          result.found = true;
          result.x = candidate.x;
          result.y = candidate.y;
          result.z = candidate.z;
          result.layer = static_cast<int>(layer);
          result.distance = distance;
        }
      }
    }
  }
  return result;
}

void PlannerCore::gridPathToWorld(const Eigen::MatrixXd& grid_path,
                                   float reference_height,
                                   PlanOutput& output) const {
  output.path.clear();
  output.layers.clear();
  if (grid_path.cols() < 3) return;
  output.path.reserve(static_cast<std::size_t>(grid_path.rows()));
  output.layers.reserve(static_cast<std::size_t>(grid_path.rows()));
  for (int i = 0; i < grid_path.rows(); ++i) {
    const int layer = static_cast<int>(std::lround(grid_path(i, 0)));
    const int row = static_cast<int>(std::lround(grid_path(i, 1)));
    const int col = static_cast<int>(std::lround(grid_path(i, 2)));
    if (layer < 0 || layer >= static_cast<int>(map_.layers) || row < 0 ||
        row >= static_cast<int>(map_.rows) || col < 0 ||
        col >= static_cast<int>(map_.cols)) {
      output.path.clear();
      output.layers.clear();
      return;
    }
    const std::size_t flat = map_.index(static_cast<uint32_t>(layer),
                                        static_cast<uint32_t>(row),
                                        static_cast<uint32_t>(col));
    // The legacy wrapper converts ground elevation with np.nan_to_num before
    // converting the raw A* result back to world coordinates.
    const float ground = legacyNanToNum(map_.ground_elevation[flat], -100.0f);
    output.path.push_back({
        (row - static_cast<int>(map_.rows / 2)) * map_.resolution +
            map_.center_x,
        (col - static_cast<int>(map_.cols / 2)) * map_.resolution +
            map_.center_y,
        ground + reference_height});
    output.layers.push_back(layer);
  }
  output.found = !output.path.empty();
}

void PlannerCore::optimizedPathToWorld(float reference_height,
                                       PlanOutput& output) const {
  output.path.clear();
  output.layers.clear();
  if (!native_) return;
  Eigen::MatrixXd trajectory;
  Eigen::VectorXd layers;
  Eigen::VectorXd heights;
  if (use_quintic_) {
    const auto& optimizer = native_->planner.get_trajectory_optimizer_wnoj();
    trajectory = optimizer.GetResultMatrix();
    layers = optimizer.GetResultLayers();
    heights = optimizer.GetResultHeight();
  } else {
    const auto& optimizer = native_->planner.get_trajectory_optimizer();
    trajectory = optimizer.GetResultMatrix();
    layers = optimizer.GetResultLayers();
    heights = optimizer.GetResultHeight();
  }
  if (trajectory.rows() == 0 || trajectory.rows() != layers.size() ||
      trajectory.rows() != heights.size()) {
    return;
  }
  const int y_column = use_quintic_ ? 3 : 2;
  if (trajectory.cols() <= y_column) return;
  // transTrajGrid2Map() uses integer floor offsets (// 2), not a geometric
  // half-size.  The distinction is one half cell for odd-sized maps.
  const double row_offset = static_cast<double>(map_.rows / 2);
  const double col_offset = static_cast<double>(map_.cols / 2);
  output.path.reserve(static_cast<std::size_t>(trajectory.rows()));
  output.layers.reserve(static_cast<std::size_t>(trajectory.rows()));
  for (int i = 0; i < trajectory.rows(); ++i) {
    const double x = trajectory(i, 0);
    const double y = trajectory(i, y_column);
    const double height = heights(i);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(height)) {
      output.path.clear();
      output.layers.clear();
      return;
    }
    // GPMP uses x=column and y=row.  The public/map convention is
    // world-x=row and world-y=column, matching transTrajGrid2Map() in the
    // legacy Python wrapper.  Keep the row/column offsets paired with the
    // corresponding map centers; mixing these two was a visible path shift
    // for non-square maps and rotated trajectories.
    output.path.push_back({
        static_cast<float>((y - row_offset) * map_.resolution +
                           map_.center_x),
        static_cast<float>((x - col_offset) * map_.resolution +
                           map_.center_y),
        static_cast<float>(height + reference_height)});
    output.layers.push_back(static_cast<int>(std::lround(layers(i))));
  }
  output.found = output.path.size() >= 2;
}

bool PlannerCore::plan(const SnapResult& start, const SnapResult& goal,
                       float reference_height, bool optimize,
                       PlanOutput& output) {
  output = PlanOutput();
  if (!loaded_ || !start.found || !goal.found || !native_) return false;
  int start_row = 0;
  int start_col = 0;
  int goal_row = 0;
  int goal_col = 0;
  if (!posToGrid(start.x, start.y, start_row, start_col) ||
      !posToGrid(goal.x, goal.y, goal_row, goal_col)) {
    return false;
  }
  if (start.layer < 0 || goal.layer < 0 ||
      start.layer >= static_cast<int>(map_.layers) ||
      goal.layer >= static_cast<int>(map_.layers)) {
    return false;
  }
  // OfflineElePlanner is copied from the legacy package. Its public index
  // convention is [layer, col, row], while Astar::GetResultMatrix() returns
  // [layer, row, col]. Use this same native core for both raw A* and GPMP so
  // there is only one path-search implementation in the new package.
  const Eigen::Vector3i native_start(start.layer, start_col, start_row);
  const Eigen::Vector3i native_goal(goal.layer, goal_col, goal_row);
  if (!native_->planner.Plan(native_start, native_goal, optimize)) return false;
  if (!optimize) {
    gridPathToWorld(native_->planner.get_path_finder().GetResultMatrix(),
                    reference_height, output);
    return output.found;
  }
  optimizedPathToWorld(reference_height, output);
  return output.found;
}

void PlannerCore::setDynamicSnapshot(
    std::shared_ptr<const DynamicSnapshot> snapshot) {
  if (snapshot && snapshot->cost.size() != map_.cellCount()) {
    throw std::invalid_argument("dynamic snapshot size does not match PCT map");
  }
  if (snapshot && snapshot->lethal_cost <= threshold_) {
    throw std::invalid_argument(
        "dynamic snapshot lethal cost must exceed PCT threshold");
  }
  dynamic_snapshot_ = std::move(snapshot);
  if (!dynamic_snapshot_) {
    if (native_) native_->planner.ClearDynamicCostMap();
    return;
  }
  if (native_) {
    native_->planner.SetDynamicCostMap(dynamic_snapshot_->cost.data(),
                                       dynamic_snapshot_->cost.size(),
                                       dynamic_snapshot_->lethal_cost);
  }
}

void PlannerCore::clearDynamicSnapshot() { setDynamicSnapshot(nullptr); }

}  // namespace global_pct_planner
