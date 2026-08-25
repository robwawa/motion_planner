#include <plan_env/pct_terrain_map.h>

#include <cmath>
#include <limits>

namespace {

int roundHalfToEven(double value) {
  const double lower_value = std::floor(value);
  const double fraction = value - lower_value;
  const int lower = static_cast<int>(lower_value);

  if (fraction < 0.5)
    return lower;
  if (fraction > 0.5)
    return lower + 1;
  return (lower % 2 == 0) ? lower : lower + 1;
}

}  // namespace

std::size_t PctTerrainMap::address(int layer, int row, int col) const {
  return (static_cast<std::size_t>(layer) * rows_ + row) * cols_ + col;
}

const char* PctTerrainMap::queryStatusName(QueryStatus status) {
  switch (status) {
    case QueryStatus::kFree: return "free";
    case QueryStatus::kOutOfMap: return "out_of_map";
    case QueryStatus::kNoElevation: return "no_elevation";
    case QueryStatus::kCostTooHigh: return "cost_too_high";
    case QueryStatus::kHeightMismatch: return "height_mismatch";
    case QueryStatus::kInvalidMap: return "invalid_map";
  }
  return "unknown";
}

bool PctTerrainMap::setFromMessage(const pct_planner::PctTerrainMap& msg,
                                   std::string& error) {
  valid_ = false;
  if (msg.resolution <= 0.0 || msg.rows == 0 || msg.cols == 0 || msg.layers == 0) {
    error = "non-positive terrain map dimensions or resolution";
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(msg.rows) * msg.cols * msg.layers;
  if (msg.traversability.size() != count || msg.ground_elevation.size() != count ||
      msg.elevation_valid.size() != count) {
    error = "terrain map array size does not match layers*rows*cols";
    return false;
  }
  resolution_ = msg.resolution;
  center_x_ = msg.center_x;
  center_y_ = msg.center_y;
  rows_ = static_cast<int>(msg.rows);
  cols_ = static_cast<int>(msg.cols);
  layers_ = static_cast<int>(msg.layers);
  frame_id_ = msg.header.frame_id;
  traversability_ = msg.traversability;
  ground_elevation_ = msg.ground_elevation;
  elevation_valid_ = msg.elevation_valid;
  valid_ = true;
  return true;
}

PctTerrainMap::QueryStatus PctTerrainMap::queryTraversableLayer(
    double x, double y, double desired_ground_z, double max_height_error,
    double traversability_threshold, Cell& result) const {
  if (!valid_)
    return QueryStatus::kInvalidMap;

  // Match NumPy np.round() used by the PCT global planner: exact half values
  // round to the nearest even grid index. This keeps SCAN on the same PCT
  // cell as the reference path at half-grid boundaries.
  const int row = roundHalfToEven((x - center_x_) / resolution_) + rows_ / 2;
  const int col = roundHalfToEven((y - center_y_) / resolution_) + cols_ / 2;
  if (row < 0 || row >= rows_ || col < 0 || col >= cols_)
    return QueryStatus::kOutOfMap;

  bool saw_elevation = false;
  bool saw_traversable_layer = false;
  double best_error = std::numeric_limits<double>::infinity();
  Cell best;
  for (int layer = 0; layer < layers_; ++layer) {
    const std::size_t index = address(layer, row, col);
    if (!elevation_valid_[index])
      continue;
    saw_elevation = true;
    if (traversability_[index] > traversability_threshold)
      continue;
    saw_traversable_layer = true;
    const double error = std::abs(static_cast<double>(ground_elevation_[index]) - desired_ground_z);
    if (error < best_error) {
      best_error = error;
      best.layer = layer;
      best.traversability = traversability_[index];
      best.ground_z = ground_elevation_[index];
    }
  }
  if (!saw_elevation)
    return QueryStatus::kNoElevation;
  if (!saw_traversable_layer)
    return QueryStatus::kCostTooHigh;
  if (best_error > max_height_error)
    return QueryStatus::kHeightMismatch;
  result = best;
  return QueryStatus::kFree;
}
