#include "global_pct_planner/tomogram.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <stdexcept>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace global_pct_planner {
namespace {

using Clock = std::chrono::steady_clock;

inline bool finitePoint(const PointXYZ& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

struct Record {
  std::size_t cell{0};
  int first_slice{0};
  float z{0.0f};
};

}  // namespace

TomogramData TomogramEngine::generate(const std::vector<PointXYZ>& points,
                                      TomogramTimings* timings) const {
  const auto total_start = Clock::now();
  if (profile_.resolution <= 0.0 || profile_.slice_dh <= 0.0 ||
      profile_.kernel_size < 1 || profile_.interval_min <= 0.0 ||
      profile_.interval_free < profile_.interval_min ||
      profile_.slope_max < 0.0 || profile_.step_max < 0.0 ||
      profile_.standable_ratio < 0.0 || profile_.standable_ratio > 1.0 ||
      profile_.cost_barrier <= 0.0 || profile_.safe_margin < 0.0 ||
      profile_.inflation < 0.0 || profile_.cost_threshold < 0.0) {
    throw std::invalid_argument("invalid PCT tomogram profile");
  }

  PointXYZ points_min{std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity()};
  PointXYZ points_max{-std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()};
  std::size_t finite_count = 0;
  for (const PointXYZ& point : points) {
    if (!finitePoint(point)) continue;
    ++finite_count;
    points_min.x = std::min(points_min.x, point.x);
    points_min.y = std::min(points_min.y, point.y);
    points_min.z = std::min(points_min.z, point.z);
    points_max.x = std::max(points_max.x, point.x);
    points_max.y = std::max(points_max.y, point.y);
    points_max.z = std::max(points_max.z, point.z);
  }

  TomogramData output;
  if (finite_count == 0) {
    output.resolution = static_cast<float>(profile_.resolution);
    output.slice_dh = static_cast<float>(profile_.slice_dh);
    output.layers = 1;
    output.rows = 4;
    output.cols = 4;
    const std::size_t count = output.cellCount();
    output.traversability.assign(
        count, static_cast<float>(profile_.cost_barrier));
    output.traversability_grad_x.assign(count, 0.0f);
    output.traversability_grad_y.assign(count, 0.0f);
    output.ground_elevation.assign(count, std::numeric_limits<float>::quiet_NaN());
    output.ceiling_elevation.assign(count, std::numeric_limits<float>::quiet_NaN());
    if (timings) {
      timings->total_ms = std::chrono::duration<double, std::milli>(
                              Clock::now() - total_start)
                              .count();
    }
    return output;
  }

  points_min.z = profile_.ground_h;
  output.resolution = static_cast<float>(profile_.resolution);
  output.slice_dh = static_cast<float>(profile_.slice_dh);
  output.center_x = 0.5f * (points_max.x + points_min.x);
  output.center_y = 0.5f * (points_max.y + points_min.y);
  const auto extentCells = [](float extent, float resolution,
                              uint32_t padding) -> uint32_t {
    const double cells = std::ceil(
        std::max(0.0, static_cast<double>(extent)) / resolution);
    if (cells > static_cast<double>(std::numeric_limits<uint32_t>::max() -
                                    padding)) {
      throw std::length_error("PCT tomogram dimensions exceed uint32 range");
    }
    return static_cast<uint32_t>(cells) + padding;
  };
  output.rows = extentCells(points_max.x - points_min.x, profile_.resolution, 4);
  output.cols = extentCells(points_max.y - points_min.y, profile_.resolution, 4);
  const uint32_t layer_count = extentCells(
      points_max.z - points_min.z, profile_.slice_dh, 0);
  output.layers = std::max<uint32_t>(1, layer_count);
  output.slice_h0 = static_cast<float>(points_min.z + profile_.slice_dh);

  const std::size_t cell_count = static_cast<std::size_t>(output.rows) * output.cols;
  const std::size_t volume = output.cellCount();
  std::vector<float> slice_heights(output.layers);
  for (uint32_t layer = 0; layer < output.layers; ++layer) {
    slice_heights[layer] = output.slice_h0 + layer * output.slice_dh;
  }

  const auto map_start = Clock::now();
  std::vector<std::size_t> point_cells(points.size(), 0);
  std::vector<int> point_first(points.size(), 0);
  std::vector<float> point_z(points.size(), 0.0f);
  std::vector<uint8_t> point_valid(points.size(), 0);

#pragma omp parallel for if(points.size() > 10000)
  for (std::int64_t point_index = 0;
       point_index < static_cast<std::int64_t>(points.size()); ++point_index) {
    const PointXYZ& point = points[static_cast<std::size_t>(point_index)];
    if (!finitePoint(point)) continue;
    const int row = static_cast<int>(std::round(
        (point.x - output.center_x) / output.resolution)) + output.rows / 2;
    const int col = static_cast<int>(std::round(
        (point.y - output.center_y) / output.resolution)) + output.cols / 2;
    if (row < 0 || row >= static_cast<int>(output.rows) || col < 0 ||
        col >= static_cast<int>(output.cols)) {
      continue;
    }
    const auto lower = std::lower_bound(slice_heights.begin(), slice_heights.end(),
                                        point.z);
    point_first[static_cast<std::size_t>(point_index)] =
        static_cast<int>(lower - slice_heights.begin());
    point_cells[static_cast<std::size_t>(point_index)] =
        static_cast<std::size_t>(row) * output.cols + col;
    point_z[static_cast<std::size_t>(point_index)] = point.z;
    point_valid[static_cast<std::size_t>(point_index)] = 1;
  }

  std::vector<Record> records;
  records.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (point_valid[i]) records.push_back({point_cells[i], point_first[i], point_z[i]});
  }

  std::vector<std::size_t> counts(cell_count, 0);
  for (const Record& record : records) ++counts[record.cell];
  std::vector<std::size_t> offsets(cell_count + 1, 0);
  for (std::size_t cell = 0; cell < cell_count; ++cell) {
    offsets[cell + 1] = offsets[cell] + counts[cell];
  }
  std::vector<std::size_t> cursor = offsets;
  std::vector<Record> bucketed(records.size());
  for (const Record& record : records) bucketed[cursor[record.cell]++] = record;

  std::vector<float> ground_at_first(volume, -1e6f);
  std::vector<float> ceiling_at_last(volume, 1e6f);
  std::vector<float> layers_g(volume, -1e6f);
  std::vector<float> layers_c(volume, 1e6f);

#pragma omp parallel for if(cell_count > 10000)
  for (std::int64_t cell_index = 0;
       cell_index < static_cast<std::int64_t>(cell_count); ++cell_index) {
    const std::size_t cell = static_cast<std::size_t>(cell_index);
    for (std::size_t k = offsets[cell]; k < offsets[cell + 1]; ++k) {
      const Record& record = bucketed[k];
      if (record.first_slice < static_cast<int>(output.layers)) {
        float& ground = ground_at_first[
            static_cast<std::size_t>(record.first_slice) * cell_count + cell];
        ground = std::max(ground, record.z);
      }
      if (record.first_slice > 0) {
        const int last = std::min(record.first_slice - 1,
                                  static_cast<int>(output.layers) - 1);
        float& ceiling =
            ceiling_at_last[static_cast<std::size_t>(last) * cell_count + cell];
        ceiling = std::min(ceiling, record.z);
      }
    }

    float current_ground = -1e6f;
    for (uint32_t layer = 0; layer < output.layers; ++layer) {
      const std::size_t index = static_cast<std::size_t>(layer) * cell_count + cell;
      current_ground = std::max(current_ground, ground_at_first[index]);
      layers_g[index] = current_ground;
    }
    float current_ceiling = 1e6f;
    for (int layer = static_cast<int>(output.layers) - 1; layer >= 0; --layer) {
      const std::size_t index = static_cast<std::size_t>(layer) * cell_count + cell;
      current_ceiling = std::min(current_ceiling, ceiling_at_last[index]);
      layers_c[index] = current_ceiling;
    }
  }

  // Match the Python CPU backend: profile scalars and slope thresholds are
  // double precision, while sampled map arrays remain float32.
  const double step_stand =
      1.2 * profile_.resolution * std::tan(profile_.slope_max);
  const double step_cross = profile_.step_max;
  // NumPy performs these operations against float32 map arrays.  Its scalar
  // promotion rules therefore use float32 thresholds for the comparisons and
  // cost arithmetic below, even though the profile values are Python doubles.
  const float step_stand_sq = static_cast<float>(step_stand * step_stand);
  const float step_cross_sq = static_cast<float>(step_cross * step_cross);
  const float interval_min = static_cast<float>(profile_.interval_min);
  const float interval_free = static_cast<float>(profile_.interval_free);
  std::vector<float> grad_mag_sq(volume, 0.0f);
  std::vector<float> grad_mag_max(volume, 0.0f);

#pragma omp parallel for if(volume > 10000)
  for (std::int64_t flat = 0; flat < static_cast<std::int64_t>(volume); ++flat) {
    const std::size_t index = static_cast<std::size_t>(flat);
    const std::size_t layer_size = cell_count;
    const int row = static_cast<int>((index % layer_size) / output.cols);
    const int col = static_cast<int>(index % output.cols);
    if (row == 0 || row + 1 >= static_cast<int>(output.rows) || col == 0 ||
        col + 1 >= static_cast<int>(output.cols)) {
      continue;
    }
    const float center = layers_g[index];
    const float dx = std::max(
        (center - layers_g[index - output.cols]) *
            (center - layers_g[index - output.cols]),
        (center - layers_g[index + output.cols]) *
            (center - layers_g[index + output.cols]));
    const float dy = std::max(
        (center - layers_g[index - 1]) * (center - layers_g[index - 1]),
        (center - layers_g[index + 1]) * (center - layers_g[index + 1]));
    grad_mag_sq[index] = dx + dy;
    grad_mag_max[index] = std::max(dx, dy);
  }
  const auto map_end = Clock::now();

  const auto trav_start = Clock::now();
  std::vector<float> trav_cost(volume, 0.0f);
  const int half_kernel = profile_.kernel_size / 2;
  const int standable_threshold = static_cast<int>(
      profile_.standable_ratio * (2 * half_kernel + 1) *
      (2 * half_kernel + 1)) - 1;

#pragma omp parallel for if(volume > 10000)
  for (std::int64_t flat = 0; flat < static_cast<std::int64_t>(volume); ++flat) {
    const std::size_t index = static_cast<std::size_t>(flat);
    const float interval = layers_c[index] - layers_g[index];
    bool barrier = interval < interval_min;
    float cost = std::max(0.0f, 20.0f * (interval_free - interval));
    const bool standable = grad_mag_sq[index] <= step_stand_sq;
    const bool crossable = grad_mag_max[index] <= step_cross_sq;
    if (standable) {
      if (step_stand_sq > 0.0f) {
        cost += 15.0f * grad_mag_sq[index] / step_stand_sq;
      }
    } else if (!crossable) {
      barrier = true;
    } else {
      const std::size_t cell = index % cell_count;
      const int row = static_cast<int>(cell / output.cols);
      const int col = static_cast<int>(cell % output.cols);
      int standable_count = 0;
      for (int dr = -half_kernel; dr <= half_kernel; ++dr) {
        for (int dc = -half_kernel; dc <= half_kernel; ++dc) {
          const int rr = row + dr;
          const int cc = col + dc;
          if (rr < 0 || rr >= static_cast<int>(output.rows) || cc < 0 ||
              cc >= static_cast<int>(output.cols)) {
            continue;
          }
          const std::size_t neighbor =
              (index / cell_count) * cell_count +
              static_cast<std::size_t>(rr) * output.cols + cc;
          if (grad_mag_sq[neighbor] < step_stand_sq) ++standable_count;
        }
      }
      if (standable_count < standable_threshold) {
        barrier = true;
      } else if (step_cross_sq > 0.0f) {
        cost += 20.0f * grad_mag_max[index] / step_cross_sq;
      }
    }
    trav_cost[index] = barrier ? static_cast<float>(profile_.cost_barrier) : cost;
  }

  struct InflationOffset {
    int dr;
    int dc;
    float weight;
  };
  // Keep the same truncation semantics as the legacy Python implementation,
  // whose profile values are Python doubles.
  const int half_inflation = static_cast<int>(
      (profile_.safe_margin + profile_.inflation) / profile_.resolution);
  std::vector<InflationOffset> inflation_offsets;
  // NumPy applies these scalar operations to float32 arrays and produces
  // float32 weights. Keep that boundary explicit even though the profile is
  // retained as double for threshold comparisons.
  const float resolution = static_cast<float>(profile_.resolution);
  const float inflation = static_cast<float>(profile_.inflation);
  const float denominator = static_cast<float>(
      profile_.safe_margin + profile_.resolution);
  for (int dr = -half_inflation; dr <= half_inflation; ++dr) {
    for (int dc = -half_inflation; dc <= half_inflation; ++dc) {
      const float distance = std::hypot(dr * resolution, dc * resolution);
      const float weight = std::max(
          0.0f, std::min(1.0f, 1.0f -
              (distance - inflation) / denominator));
      if (weight > 0.0f) inflation_offsets.push_back({dr, dc, weight});
    }
  }
  std::vector<float> inflated(volume, 0.0f);

#pragma omp parallel for if(volume > 10000)
  for (std::int64_t flat = 0; flat < static_cast<std::int64_t>(volume); ++flat) {
    const std::size_t index = static_cast<std::size_t>(flat);
    const std::size_t cell = index % cell_count;
    const int row = static_cast<int>(cell / output.cols);
    const int col = static_cast<int>(cell % output.cols);
    const std::size_t layer_base = index - cell;
    float maximum_log = -std::numeric_limits<float>::infinity();
    for (const InflationOffset& offset : inflation_offsets) {
      const int rr = row + offset.dr;
      const int cc = col + offset.dc;
      if (rr < 0 || rr >= static_cast<int>(output.rows) || cc < 0 ||
          cc >= static_cast<int>(output.cols)) {
        continue;
      }
      const std::size_t neighbor = layer_base +
          static_cast<std::size_t>(rr) * output.cols + cc;
      const float source_cost = trav_cost[neighbor];
      if (source_cost > 0.0f) {
        // NumPy's log/grey_dilation path is float32 throughout.  Keep each
        // logarithm and the reduction operand at float32 as well instead of
        // widening the log-domain reduction to double.
        const float candidate =
            static_cast<float>(std::log(source_cost)) +
            static_cast<float>(std::log(offset.weight));
        maximum_log = std::max(
            maximum_log, candidate);
      }
    }
    inflated[index] = std::exp(maximum_log);
  }
  const auto trav_end = Clock::now();

  const auto simplify_start = Clock::now();
  std::vector<int> selected_layers;
  selected_layers.push_back(0);
  if (output.layers > 1) {
    int lower = 0;
    int middle = 1;
    while (middle < static_cast<int>(output.layers) - 2) {
      bool unique = false;
      for (std::size_t cell = 0; cell < cell_count; ++cell) {
        const std::size_t middle_index = static_cast<std::size_t>(middle) * cell_count + cell;
        const std::size_t lower_index = static_cast<std::size_t>(lower) * cell_count + cell;
        const std::size_t upper_index = static_cast<std::size_t>(middle + 1) * cell_count + cell;
        if (((layers_g[middle_index] - layers_g[lower_index] > 0.0f) ||
             (inflated[lower_index] > inflated[middle_index])) &&
            (layers_g[upper_index] - layers_g[middle_index] > 0.0f) &&
            inflated[middle_index] < profile_.cost_barrier) {
          unique = true;
          break;
        }
      }
      if (unique) {
        selected_layers.push_back(middle);
        lower = middle;
      }
      ++middle;
    }
    selected_layers.push_back(middle);
  }

  output.layers = static_cast<uint32_t>(selected_layers.size());
  const std::size_t selected_volume = output.cellCount();
  output.traversability.assign(selected_volume, 0.0f);
  output.traversability_grad_x.assign(selected_volume, 0.0f);
  output.traversability_grad_y.assign(selected_volume, 0.0f);
  output.ground_elevation.assign(selected_volume,
                                 std::numeric_limits<float>::quiet_NaN());
  output.ceiling_elevation.assign(selected_volume,
                                  std::numeric_limits<float>::quiet_NaN());
  for (std::size_t selected = 0; selected < selected_layers.size(); ++selected) {
    const int source_layer = selected_layers[selected];
    for (std::size_t cell = 0; cell < cell_count; ++cell) {
      const std::size_t source = static_cast<std::size_t>(source_layer) * cell_count + cell;
      const std::size_t target = selected * cell_count + cell;
      output.traversability[target] = inflated[source];
      if (layers_g[source] > -1e6f) output.ground_elevation[target] = layers_g[source];
      if (layers_c[source] < 1e6f) output.ceiling_elevation[target] = layers_c[source];
    }
    for (uint32_t row = 1; row + 1 < output.rows; ++row) {
      for (uint32_t col = 0; col < output.cols; ++col) {
        const std::size_t target = selected * cell_count +
            static_cast<std::size_t>(row) * output.cols + col;
        output.traversability_grad_x[target] =
            output.traversability[target + output.cols] -
            output.traversability[target - output.cols];
      }
    }
    for (uint32_t row = 0; row < output.rows; ++row) {
      for (uint32_t col = 1; col + 1 < output.cols; ++col) {
        const std::size_t target = selected * cell_count +
            static_cast<std::size_t>(row) * output.cols + col;
        output.traversability_grad_y[target] =
            output.traversability[target + 1] - output.traversability[target - 1];
      }
    }
  }
  const auto simplify_end = Clock::now();

  if (timings) {
    timings->map_ms = std::chrono::duration<double, std::milli>(map_end - map_start).count();
    timings->traversability_ms = std::chrono::duration<double, std::milli>(
        trav_end - trav_start).count();
    timings->simplification_ms = std::chrono::duration<double, std::milli>(
        simplify_end - simplify_start).count();
    timings->total_ms = std::chrono::duration<double, std::milli>(
        simplify_end - total_start).count();
  }
  return output;
}

}  // namespace global_pct_planner
