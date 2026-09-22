#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace global_pct_planner {

struct PointXYZ {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct TomogramProfile {
  // The legacy Python profile stores these scalars as double precision
  // values. Keep the profile at double precision and cast only at the
  // float32 map-array boundaries.
  double resolution{0.1};
  double ground_h{0.0};
  double slice_dh{0.5};
  int kernel_size{7};
  double interval_min{0.5};
  double interval_free{0.5};
  double slope_max{0.4};
  double step_max{0.3};
  double standable_ratio{0.5};
  double cost_barrier{50.0};
  double safe_margin{0.5};
  double inflation{0.1};
  double cost_threshold{49.0};
};

struct TomogramTimings {
  double map_ms{0.0};
  double traversability_ms{0.0};
  double simplification_ms{0.0};
  double total_ms{0.0};
};

struct TomogramData {
  float resolution{0.0f};
  float center_x{0.0f};
  float center_y{0.0f};
  float slice_h0{0.0f};
  float slice_dh{0.0f};
  uint32_t layers{0};
  uint32_t rows{0};
  uint32_t cols{0};

  // All arrays use C-order [layer][row][col].
  std::vector<float> traversability;
  std::vector<float> traversability_grad_x;
  std::vector<float> traversability_grad_y;
  std::vector<float> ground_elevation;
  std::vector<float> ceiling_elevation;

  std::size_t cellCount() const {
    return static_cast<std::size_t>(layers) * rows * cols;
  }
  std::size_t index(uint32_t layer, uint32_t row, uint32_t col) const {
    return (static_cast<std::size_t>(layer) * rows + row) * cols + col;
  }
};

class TomogramEngine {
 public:
  explicit TomogramEngine(TomogramProfile profile) : profile_(profile) {}

  TomogramData generate(const std::vector<PointXYZ>& points,
                        TomogramTimings* timings = nullptr) const;

 private:
  TomogramProfile profile_;
};

}  // namespace global_pct_planner
