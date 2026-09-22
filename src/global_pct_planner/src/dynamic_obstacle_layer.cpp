#include "global_pct_planner/dynamic_obstacle_layer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace global_pct_planner {
namespace {

bool finitePoint(const PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

// The legacy Python planner passes nan_to_num()-processed elevation arrays to
// its dynamic layer.  Keep the serialized .pctm arrays untouched, but apply
// the same conversion at this boundary before projecting dynamic points.
float legacyNanToNum(float value, float nan_replacement) {
  if (std::isnan(value)) return nan_replacement;
  if (std::isinf(value)) {
    const float max_value = std::numeric_limits<float>::max();
    return value > 0.0f ? max_value : -max_value;
  }
  return value;
}

int roundHalfToEvenDouble(double value) {
  if (!std::isfinite(value)) return 0;
  const double floor_value = std::floor(value);
  const double fraction = value - floor_value;
  if (fraction < 0.5) return static_cast<int>(floor_value);
  if (fraction > 0.5) return static_cast<int>(floor_value + 1.0);
  const auto lower = static_cast<long long>(floor_value);
  return static_cast<int>((lower % 2 == 0) ? lower : lower + 1);
}

}  // namespace

DynamicObstacleLayer::DynamicObstacleLayer(
    const TomogramData& map, float threshold, uint8_t lethal_cost,
    float terrain_ignore_height, float collision_top,
    float layer_height_tolerance, float robot_radius, float safety_margin,
    float inflation_radius, int max_points_per_snapshot,
    double max_memory_mb)
    : map_(map),
      threshold_(threshold),
      lethal_cost_(lethal_cost),
      terrain_ignore_height_(terrain_ignore_height),
      collision_top_(collision_top),
      layer_height_tolerance_(layer_height_tolerance),
      robot_radius_(robot_radius),
      safety_margin_(safety_margin),
      inflation_radius_(inflation_radius),
      max_points_per_snapshot_(std::max(1, max_points_per_snapshot)),
      layers_(map.layers),
      rows_(map.rows),
      cols_(map.cols),
      resolution_(map.resolution),
      center_x_(map.center_x),
      center_y_(map.center_y) {
  if (layers_ == 0 || rows_ == 0 || cols_ == 0 || resolution_ <= 0.0f) {
    throw std::invalid_argument("dynamic layer received an empty PCT map");
  }
  const std::size_t map_cells = map.cellCount();
  if (map.traversability.size() != map_cells ||
      map.ground_elevation.size() != map_cells ||
      map.ceiling_elevation.size() != map_cells) {
    throw std::invalid_argument("dynamic layer received incomplete PCT arrays");
  }
  if (lethal_cost_ == 0 || lethal_cost_ <= threshold_) {
    throw std::invalid_argument(
        "dynamic lethal cost must be uint8 and exceed PCT threshold");
  }
  if (!std::isfinite(max_memory_mb) || max_memory_mb <= 0.0) {
    throw std::invalid_argument("dynamic memory budget must be finite and positive");
  }
  if (inflation_radius_ < robot_radius_ + safety_margin_) {
    throw std::invalid_argument(
        "dynamic inflation radius must cover robot radius and safety margin");
  }
  const std::size_t volume = map_cells;
  if (volume > std::numeric_limits<std::size_t>::max() / 6) {
    throw std::invalid_argument("dynamic PCT map is too large");
  }
  const double required_mb =
      static_cast<double>(volume * 6) / (1024.0 * 1024.0);
  if (required_mb > max_memory_mb) {
    throw std::length_error("dynamic layer exceeds configured memory budget");
  }

  occupied_.assign(volume, 0);
  occupied_z_.assign(volume, std::numeric_limits<float>::quiet_NaN());
  snapshot_ = std::make_shared<DynamicSnapshot>();
  snapshot_->cost.assign(volume, 0);
  snapshot_->version = 0;
  snapshot_->lethal_cost = lethal_cost_;

  const int radius_cells = static_cast<int>(
      std::ceil(inflation_radius_ / resolution_));
  const float inscribed = robot_radius_ + safety_margin_;
  const float sigma = std::max((inflation_radius_ - inscribed) / 2.0f,
                               resolution_);
  offsets_.reserve(static_cast<std::size_t>(2 * radius_cells + 1) *
                   static_cast<std::size_t>(2 * radius_cells + 1));
  for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
    for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
      const double distance = std::hypot(
          static_cast<double>(dr) * resolution_,
          static_cast<double>(dc) * resolution_);
      if (distance > inflation_radius_) continue;
      uint8_t cost = lethal_cost_;
      if (distance > inscribed) {
        const double normalized =
            (distance - static_cast<double>(inscribed)) / sigma;
        const double value = static_cast<double>(threshold_ - 1.0f) *
                             std::exp(-0.5 * normalized * normalized);
        // Python round() is ties-to-even.  std::lround() is ties-away-from-
        // zero and changes a configurable inflation profile at exact .5.
        const int rounded = roundHalfToEvenDouble(value);
        cost = static_cast<uint8_t>(std::max(
            1, std::min(static_cast<int>(lethal_cost_) - 1, rounded)));
      }
      offsets_.push_back({dr, dc, cost});
    }
  }
}

int DynamicObstacleLayer::roundHalfToEven(double value) {
  return roundHalfToEvenDouble(value);
}

std::pair<int, int> DynamicObstacleLayer::worldToGrid(float x, float y) const {
  const int row = roundHalfToEven((x - center_x_) / resolution_) +
                  static_cast<int>(rows_ / 2);
  const int col = roundHalfToEven((y - center_y_) / resolution_) +
                  static_cast<int>(cols_ / 2);
  return {row, col};
}

std::pair<float, float> DynamicObstacleLayer::gridToWorld(int row,
                                                            int col) const {
  return {(row - static_cast<int>(rows_ / 2)) * resolution_ + center_x_,
          (col - static_cast<int>(cols_ / 2)) * resolution_ + center_y_};
}

std::vector<AssignedDynamicPoint> DynamicObstacleLayer::assignPoints(
    const std::vector<PointXYZ>& points) const {
  std::vector<PointXYZ> filtered;
  filtered.reserve(std::min<std::size_t>(points.size(),
                                         static_cast<std::size_t>(
                                             max_points_per_snapshot_)));
  for (const PointXYZ& point : points) {
    if (finitePoint(point)) filtered.push_back(point);
  }
  if (filtered.size() > static_cast<std::size_t>(max_points_per_snapshot_)) {
    const std::size_t stride = static_cast<std::size_t>(std::ceil(
        filtered.size() / static_cast<double>(max_points_per_snapshot_)));
    std::vector<PointXYZ> sampled;
    sampled.reserve(max_points_per_snapshot_);
    for (std::size_t i = 0; i < filtered.size(); i += stride) {
      sampled.push_back(filtered[i]);
    }
    filtered.swap(sampled);
  }

  const std::size_t volume = map_.cellCount();
  std::vector<uint8_t> seen(volume, 0);
  std::vector<AssignedDynamicPoint> assigned;
  assigned.reserve(filtered.size());
  for (const PointXYZ& point : filtered) {
    const auto grid = worldToGrid(point.x, point.y);
    const int row = grid.first;
    const int col = grid.second;
    if (row < 0 || row >= static_cast<int>(rows_) || col < 0 ||
        col >= static_cast<int>(cols_)) {
      continue;
    }
    int selected_layer = -1;
    float selected_distance = std::numeric_limits<float>::infinity();
    for (uint32_t layer = 0; layer < layers_; ++layer) {
      const std::size_t index = map_.index(layer, row, col);
      const float ground = legacyNanToNum(
          map_.ground_elevation[index], -100.0f);
      const float ceiling = legacyNanToNum(
          map_.ceiling_elevation[index], 1e6f);
      const float traversability = map_.traversability[index];
      if (!std::isfinite(ground) || !std::isfinite(ceiling) ||
          !std::isfinite(traversability) || traversability > threshold_) {
        continue;
      }
      const float dz = point.z - ground;
      if (!(dz > terrain_ignore_height_ && dz <= collision_top_) ||
          point.z < ground - layer_height_tolerance_ ||
          point.z > ceiling + layer_height_tolerance_) {
        continue;
      }
      if (std::abs(dz) < selected_distance) {
        selected_distance = std::abs(dz);
        selected_layer = static_cast<int>(layer);
      }
    }
    if (selected_layer < 0) continue;
    const std::size_t index = map_.index(selected_layer, row, col);
    if (seen[index]) continue;
    seen[index] = 1;
    assigned.push_back({selected_layer, row, col, point.z});
  }
  return assigned;
}

bool DynamicObstacleLayer::stateEquals(
    const std::vector<uint8_t>& occupied,
    const std::vector<float>& occupied_z) const {
  if (occupied != occupied_) return false;
  if (occupied_z.size() != occupied_z_.size()) return false;
  for (std::size_t i = 0; i < occupied_z.size(); ++i) {
    if (occupied_z[i] == occupied_z_[i]) continue;
    if (std::isnan(occupied_z[i]) && std::isnan(occupied_z_[i])) continue;
    return false;
  }
  return true;
}

bool DynamicObstacleLayer::replaceSnapshot(const std::vector<PointXYZ>& points) {
  const std::vector<AssignedDynamicPoint> assigned = assignPoints(points);
  std::vector<uint8_t> next_occupied(map_.cellCount(), 0);
  std::vector<float> next_z(map_.cellCount(),
                            std::numeric_limits<float>::quiet_NaN());
  for (const AssignedDynamicPoint& point : assigned) {
    const std::size_t index = map_.index(point.layer, point.row, point.col);
    next_occupied[index] = 1;
    next_z[index] = point.z;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  assigned_points_ = assigned;
  if (stateEquals(next_occupied, next_z)) return false;
  occupied_.swap(next_occupied);
  occupied_z_.swap(next_z);
  rebuildCostLocked();
  return true;
}

bool DynamicObstacleLayer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool changed = std::any_of(occupied_.begin(), occupied_.end(),
                                   [](uint8_t value) { return value != 0; }) ||
                       std::any_of(snapshot_->cost.begin(), snapshot_->cost.end(),
                                   [](uint8_t value) { return value != 0; });
  if (!changed) return false;
  std::fill(occupied_.begin(), occupied_.end(), 0);
  std::fill(occupied_z_.begin(), occupied_z_.end(),
            std::numeric_limits<float>::quiet_NaN());
  assigned_points_.clear();
  snapshot_ = std::make_shared<DynamicSnapshot>();
  snapshot_->cost.assign(map_.cellCount(), 0);
  snapshot_->version = version_ + 1;
  snapshot_->lethal_cost = lethal_cost_;
  version_ = snapshot_->version;
  return true;
}

void DynamicObstacleLayer::rebuildCostLocked() {
  std::shared_ptr<DynamicSnapshot> next = std::make_shared<DynamicSnapshot>();
  next->cost.assign(map_.cellCount(), 0);
  for (uint32_t layer = 0; layer < layers_; ++layer) {
    for (uint32_t row = 0; row < rows_; ++row) {
      for (uint32_t col = 0; col < cols_; ++col) {
        const std::size_t source = map_.index(layer, row, col);
        if (!occupied_[source]) continue;
        for (const Offset& offset : offsets_) {
          const int target_row = static_cast<int>(row) + offset.row;
          const int target_col = static_cast<int>(col) + offset.col;
          if (target_row < 0 || target_row >= static_cast<int>(rows_) ||
              target_col < 0 || target_col >= static_cast<int>(cols_)) {
            continue;
          }
          const std::size_t target =
              map_.index(layer, target_row, target_col);
          next->cost[target] = std::max(next->cost[target], offset.cost);
        }
      }
    }
  }
  next->version = version_ + 1;
  next->lethal_cost = lethal_cost_;
  version_ = next->version;
  snapshot_ = std::move(next);
}

std::shared_ptr<const DynamicSnapshot> DynamicObstacleLayer::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

std::vector<AssignedDynamicPoint> DynamicObstacleLayer::assignedPoints() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return assigned_points_;
}

std::vector<AssignedDynamicPoint> DynamicObstacleLayer::nonzeroCells(
    std::size_t max_points) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<AssignedDynamicPoint> cells;
  for (uint32_t layer = 0; layer < layers_; ++layer) {
    for (uint32_t row = 0; row < rows_; ++row) {
      for (uint32_t col = 0; col < cols_; ++col) {
        const std::size_t index = map_.index(layer, row, col);
        if (snapshot_->cost[index] == 0) continue;
        cells.push_back({static_cast<int>(layer), static_cast<int>(row),
                         static_cast<int>(col),
                         legacyNanToNum(map_.ground_elevation[index], -100.0f) +
                             0.03f});
      }
    }
  }
  if (max_points > 0 && cells.size() > max_points) {
    const std::size_t stride = static_cast<std::size_t>(std::ceil(
        cells.size() / static_cast<double>(max_points)));
    std::vector<AssignedDynamicPoint> sampled;
    sampled.reserve(max_points);
    for (std::size_t i = 0; i < cells.size(); i += stride) {
      sampled.push_back(cells[i]);
    }
    cells.swap(sampled);
  }
  return cells;
}

}  // namespace global_pct_planner
