// Frozen pre-optimization implementation, used only by regression tests.
#include "reference_cleaner.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <flann/flann.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <unistd.h>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace fs = std::filesystem;

namespace reference_pct_map_cleaner {
namespace {

struct Cell2 {
  std::int64_t x = 0;
  std::int64_t y = 0;

  bool operator==(const Cell2& other) const {
    return x == other.x && y == other.y;
  }

  bool operator<(const Cell2& other) const {
    return x == other.x ? y < other.y : x < other.x;
  }
};

struct Cell3 {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const Cell3& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator<(const Cell3& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

std::size_t hash_mix(std::uint64_t value) {
  value ^= value >> 30U;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27U;
  value *= 0x94d049bb133111ebULL;
  value ^= value >> 31U;
  return static_cast<std::size_t>(value);
}

struct Cell2Hash {
  std::size_t operator()(const Cell2& cell) const {
    const auto hx = hash_mix(static_cast<std::uint64_t>(cell.x));
    const auto hy = hash_mix(static_cast<std::uint64_t>(cell.y));
    return hx ^ (hy + 0x9e3779b97f4a7c15ULL + (hx << 6U) + (hx >> 2U));
  }
};

struct Cell3Hash {
  std::size_t operator()(const Cell3& cell) const {
    const auto hxy = Cell2Hash{}(Cell2{cell.x, cell.y});
    const auto hz = hash_mix(static_cast<std::uint64_t>(cell.z));
    return hxy ^ (hz + 0x9e3779b97f4a7c15ULL + (hxy << 6U) + (hxy >> 2U));
  }
};

std::size_t checked_reserve_size(std::size_t size, double factor = 1.3) {
  const double requested = static_cast<double>(size) * factor + 1.0;
  if (requested > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
    throw std::overflow_error("point-cloud index is too large");
  }
  return static_cast<std::size_t>(requested);
}

int worker_count(int requested) {
  if (requested > 0) return requested;
#ifdef _OPENMP
  return std::max(1, omp_get_max_threads());
#else
  return 1;
#endif
}

Cell3 point_cell(const Point3f& point, double spacing) {
  return Cell3{
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.x) / spacing)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.y) / spacing)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.z) / spacing))};
}

Cell2 point_cell_xy(const Point3f& point, double spacing) {
  return Cell2{
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.x) / spacing)),
      static_cast<std::int64_t>(std::floor(static_cast<double>(point.y) / spacing))};
}

Point3f cell_center(const Cell3& cell, double spacing) {
  return Point3f{
      static_cast<float>((static_cast<double>(cell.x) + 0.5) * spacing),
      static_cast<float>((static_cast<double>(cell.y) + 0.5) * spacing),
      static_cast<float>((static_cast<double>(cell.z) + 0.5) * spacing)};
}

double squared_distance(const Point3f& left, const Point3f& right) {
  const double dx = static_cast<double>(left.x) - right.x;
  const double dy = static_cast<double>(left.y) - right.y;
  const double dz = static_cast<double>(left.z) - right.z;
  return dx * dx + dy * dy + dz * dz;
}

template <typename Callback>
void for_each_neighbor_cell(const Cell3& cell, const Callback& callback) {
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        callback(Cell3{cell.x + dx, cell.y + dy, cell.z + dz});
      }
    }
  }
}

std::vector<Point3f> load_xyz(const fs::path& path) {
  if (!fs::is_regular_file(path)) {
    throw std::runtime_error("input PCD does not exist: " + path.string());
  }
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path.string(), cloud) != 0) {
    throw std::runtime_error("failed to read PCD: " + path.string());
  }
  std::vector<Point3f> points;
  points.reserve(cloud.size());
  for (const auto& point : cloud.points) {
    points.push_back(Point3f{point.x, point.y, point.z});
  }
  return points;
}

void write_xyz_atomic(const fs::path& output_path, const PointCloudXYZ& cloud) {
  const fs::path parent = output_path.parent_path().empty()
      ? fs::path(".") : output_path.parent_path();
  fs::create_directories(parent);
  const fs::path temporary = output_path.string() + ".tmp." +
      std::to_string(static_cast<long long>(::getpid()));

  try {
    if (cloud.points.empty()) {
      std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
      if (!stream) throw std::runtime_error("failed to open temporary PCD");
      stream << "# .PCD v0.7 - Point Cloud Data file format\n"
             << "VERSION 0.7\n"
             << "FIELDS x y z\n"
             << "SIZE 4 4 4\n"
             << "TYPE F F F\n"
             << "COUNT 1 1 1\n"
             << "WIDTH 0\n"
             << "HEIGHT 1\n"
             << "VIEWPOINT 0 0 0 1 0 0 0\n"
             << "POINTS 0\n"
             << "DATA ascii\n";
      stream.close();
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl_cloud.points.reserve(cloud.points.size());
      for (const auto& point : cloud.points) {
        pcl_cloud.points.emplace_back(point.x, point.y, point.z);
      }
      pcl_cloud.width = static_cast<std::uint32_t>(pcl_cloud.points.size());
      pcl_cloud.height = 1;
      if (pcl::io::savePCDFileBinary(temporary.string(), pcl_cloud) != 0) {
        throw std::runtime_error("failed to write temporary PCD");
      }
    }
    fs::rename(temporary, output_path);
  } catch (...) {
    std::error_code error;
    fs::remove(temporary, error);
    throw;
  }
}

struct PreGroup {
  Cell3 cell;
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  double max_representation_error = 0.0;
};

struct SupportPoint {
  double z = 0.0;
  std::size_t source_index = 0;
  Cell2 source_cell;
  int dx = 0;
  int dy = 0;
};

struct CompletionCandidate {
  Cell2 cell;
  double z = 0.0;
  std::size_t source_index = 0;
};

class DisjointSet {
public:
  explicit DisjointSet(std::size_t size) : parent_(size), rank_(size, 0) {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  std::size_t find(std::size_t value) {
    while (parent_[value] != value) {
      parent_[value] = parent_[parent_[value]];
      value = parent_[value];
    }
    return value;
  }

  void unite(std::size_t left, std::size_t right) {
    left = find(left);
    right = find(right);
    if (left == right) return;
    if (rank_[left] < rank_[right]) std::swap(left, right);
    parent_[right] = left;
    if (rank_[left] == rank_[right]) ++rank_[left];
  }

private:
  std::vector<std::size_t> parent_;
  std::vector<unsigned char> rank_;
};

}  // namespace

PctMapCleaner::PctMapCleaner(CleanerConfig config) : config_(std::move(config)) {
  validate();
}

void PctMapCleaner::validate() const {
  const auto positive = [](double value, const char* name) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be positive and finite");
    }
  };
  const auto at_least = [](int value, int minimum, const char* name) {
    if (value < minimum) {
      throw std::invalid_argument(std::string(name) + " is below its minimum");
    }
  };

  positive(config_.pre_spacing, "pre_spacing");
  positive(config_.max_pre_rep_error, "max_pre_rep_error");
  positive(config_.sor_std_ratio, "sor_std_ratio");
  positive(config_.ror_radius, "ror_radius");
  positive(config_.cluster_eps, "cluster_eps");
  positive(config_.cluster_max_bbox_x, "cluster_max_bbox_x");
  positive(config_.cluster_max_bbox_y, "cluster_max_bbox_y");
  positive(config_.cluster_max_bbox_z, "cluster_max_bbox_z");
  positive(config_.final_spacing, "final_spacing");
  positive(config_.ground_spacing, "ground_spacing");
  positive(config_.ground_support_radius, "ground_support_radius");
  positive(config_.ground_z_tolerance, "ground_z_tolerance");
  positive(config_.ground_max_hole_area, "ground_max_hole_area");

  at_least(config_.sor_neighbors, 1, "sor_neighbors");
  at_least(config_.ror_min_neighbors, 1, "ror_min_neighbors");
  at_least(config_.cluster_min_points, 1, "cluster_min_points");
  at_least(config_.cluster_max_points, 1, "cluster_max_points");
  at_least(config_.ground_min_support_cells, 1, "ground_min_support_cells");
  if (config_.ground_min_support_directions < 1 ||
      config_.ground_min_support_directions > 4) {
    throw std::invalid_argument("ground_min_support_directions must be in [1, 4]");
  }
  if (config_.threads < 0) {
    throw std::invalid_argument("threads must be >= 0");
  }
  if (config_.ground_completion_enable &&
      std::abs(config_.ground_spacing - config_.final_spacing) > 1e-9) {
    throw std::invalid_argument(
        "ground_completion.spacing must equal uniform_sampling.spacing");
  }
}

PointCloudXYZ PctMapCleaner::remove_non_finite(PointCloudXYZ input) const {
  PointCloudXYZ output;
  output.points.reserve(input.points.size());
  for (const auto& point : input.points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      output.points.push_back(point);
    }
  }
  return output;
}

PointCloudXYZ PctMapCleaner::pre_downsample(PointCloudXYZ input) const {
  if (!config_.pre_sampling_enable || input.points.empty()) return input;

  std::unordered_map<Cell3, PreGroup, Cell3Hash> grouped;
  grouped.reserve(checked_reserve_size(input.points.size()));
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    const auto cell = point_cell(input.points[index], config_.pre_spacing);
    const auto center = cell_center(cell, config_.pre_spacing);
    const double distance = squared_distance(input.points[index], center);
    auto [iterator, inserted] = grouped.emplace(
        cell, PreGroup{cell, index, distance, 0.0});
    if (!inserted) {
      auto& group = iterator->second;
      if (distance < group.best_distance ||
          (distance == group.best_distance && index < group.best_index)) {
        group.best_index = index;
        group.best_distance = distance;
      }
    }
  }

  std::vector<PreGroup> groups;
  groups.reserve(grouped.size());
  for (auto& item : grouped) groups.push_back(item.second);
  std::sort(groups.begin(), groups.end(),
            [](const PreGroup& left, const PreGroup& right) {
              return left.cell < right.cell;
            });

  std::unordered_map<Cell3, std::size_t, Cell3Hash> group_indices;
  group_indices.reserve(checked_reserve_size(groups.size()));
  for (std::size_t index = 0; index < groups.size(); ++index) {
    group_indices.emplace(groups[index].cell, index);
  }

  for (const auto& point : input.points) {
    const auto group_it = group_indices.find(point_cell(point, config_.pre_spacing));
    if (group_it == group_indices.end()) {
      throw std::logic_error("pre-sampling voxel index disappeared");
    }
    auto& group = groups[group_it->second];
    group.max_representation_error = std::max(
        group.max_representation_error,
        std::sqrt(squared_distance(point, input.points[group.best_index])));
  }

  PointCloudXYZ output;
  output.points.reserve(groups.size());
  for (const auto& group : groups) {
    if (group.max_representation_error > config_.max_pre_rep_error) {
      throw std::runtime_error(
          "pre-sampling representative error exceeds max_pre_rep_error");
    }
    output.points.push_back(input.points[group.best_index]);
  }
  return output;
}

PointCloudXYZ PctMapCleaner::statistical_filter(PointCloudXYZ input) const {
  if (!config_.sor_enable || input.points.size() <=
      static_cast<std::size_t>(config_.sor_neighbors)) {
    return input;
  }

  const std::size_t count = input.points.size();
  const std::size_t query_count = static_cast<std::size_t>(config_.sor_neighbors) + 1U;
  std::vector<float> data(count * 3U);
  for (std::size_t index = 0; index < count; ++index) {
    data[index * 3U] = input.points[index].x;
    data[index * 3U + 1U] = input.points[index].y;
    data[index * 3U + 2U] = input.points[index].z;
  }

  flann::Matrix<float> dataset(data.data(), count, 3);
  flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(4));
  index.buildIndex();
  std::vector<double> mean_distances(count, 0.0);
  const int workers = worker_count(config_.threads);

#pragma omp parallel num_threads(workers)
  {
    std::vector<int> neighbor_indices(query_count);
    std::vector<float> neighbor_distances(query_count);
#pragma omp for schedule(static)
    for (std::int64_t item = 0;
         item < static_cast<std::int64_t>(count); ++item) {
      const std::size_t point_index = static_cast<std::size_t>(item);
      flann::Matrix<float> query(&data[point_index * 3U], 1, 3);
      flann::Matrix<int> result_indices(neighbor_indices.data(), 1, query_count);
      flann::Matrix<float> result_distances(
          neighbor_distances.data(), 1, query_count);
      const int found = index.knnSearch(
          query, result_indices, result_distances, query_count,
          flann::SearchParams(64));
      double total = 0.0;
      int used = 0;
      for (int neighbor = 0; neighbor < found; ++neighbor) {
        if (neighbor_indices[neighbor] == static_cast<int>(point_index)) continue;
        total += std::sqrt(std::max(0.0F, neighbor_distances[neighbor]));
        ++used;
        if (used == config_.sor_neighbors) break;
      }
      mean_distances[point_index] = used == 0 ? 0.0 : total / used;
    }
  }

  const double mean = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0) /
      static_cast<double>(count);
  double variance = 0.0;
  for (const double value : mean_distances) {
    const double difference = value - mean;
    variance += difference * difference;
  }
  variance /= static_cast<double>(count);
  const double threshold = mean + config_.sor_std_ratio * std::sqrt(variance);

  PointCloudXYZ output;
  output.points.reserve(input.points.size());
  for (std::size_t index = 0; index < count; ++index) {
    if (mean_distances[index] <= threshold) output.points.push_back(input.points[index]);
  }
  return output;
}

PointCloudXYZ PctMapCleaner::radius_filter(PointCloudXYZ input) const {
  if (!config_.ror_enable || input.points.empty()) return input;

  using Grid = std::unordered_map<Cell3, std::vector<std::size_t>, Cell3Hash>;
  Grid grid;
  grid.reserve(checked_reserve_size(input.points.size()));
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    grid[point_cell(input.points[index], config_.ror_radius)].push_back(index);
  }

  const double radius_squared = config_.ror_radius * config_.ror_radius;
  std::vector<unsigned char> keep(input.points.size(), 0U);
  const int workers = worker_count(config_.threads);
#pragma omp parallel for num_threads(workers) schedule(static)
  for (std::int64_t item = 0;
       item < static_cast<std::int64_t>(input.points.size()); ++item) {
    const std::size_t point_index = static_cast<std::size_t>(item);
    const auto cell = point_cell(input.points[point_index], config_.ror_radius);
    std::size_t neighbors = 0;
    for_each_neighbor_cell(cell, [&](const Cell3& neighbor_cell) {
      const auto iterator = grid.find(neighbor_cell);
      if (iterator == grid.end()) return;
      for (const std::size_t candidate : iterator->second) {
        if (candidate == point_index) continue;
        if (squared_distance(input.points[point_index], input.points[candidate]) <=
            radius_squared) {
          ++neighbors;
        }
      }
    });
    keep[point_index] = neighbors >=
        static_cast<std::size_t>(config_.ror_min_neighbors) ? 1U : 0U;
  }

  PointCloudXYZ output;
  output.points.reserve(input.points.size());
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    if (keep[index] != 0U) output.points.push_back(input.points[index]);
  }
  return output;
}

PointCloudXYZ PctMapCleaner::floating_cluster_filter(PointCloudXYZ input) const {
  if (!config_.floating_cluster_enable || input.points.empty()) return input;

  using Grid = std::unordered_map<Cell3, std::vector<std::size_t>, Cell3Hash>;
  Grid grid;
  grid.reserve(checked_reserve_size(input.points.size()));
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    grid[point_cell(input.points[index], config_.cluster_eps)].push_back(index);
  }

  const double radius_squared = config_.cluster_eps * config_.cluster_eps;
  std::vector<unsigned char> core(input.points.size(), 0U);
  const int workers = worker_count(config_.threads);
#pragma omp parallel for num_threads(workers) schedule(static)
  for (std::int64_t item = 0;
       item < static_cast<std::int64_t>(input.points.size()); ++item) {
    const std::size_t point_index = static_cast<std::size_t>(item);
    const auto cell = point_cell(input.points[point_index], config_.cluster_eps);
    std::size_t neighbors = 0;
    for_each_neighbor_cell(cell, [&](const Cell3& neighbor_cell) {
      const auto iterator = grid.find(neighbor_cell);
      if (iterator == grid.end()) return;
      for (const std::size_t candidate : iterator->second) {
        if (squared_distance(input.points[point_index], input.points[candidate]) <=
            radius_squared) {
          ++neighbors;
        }
      }
    });
    core[point_index] = neighbors >=
        static_cast<std::size_t>(config_.cluster_min_points) ? 1U : 0U;
  }

  std::vector<int> labels(input.points.size(), -1);
  struct ClusterStats {
    std::size_t count = 0;
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
  };
  std::vector<ClusterStats> clusters;
  std::vector<std::size_t> queue;
  std::vector<std::size_t> neighbors;
  std::vector<unsigned char> queued_core(input.points.size(), 0U);

  for (std::size_t seed = 0; seed < input.points.size(); ++seed) {
    if (core[seed] == 0U || labels[seed] >= 0) continue;
    const int label = static_cast<int>(clusters.size());
    clusters.emplace_back();
    queue.clear();
    queue.push_back(seed);
    queued_core[seed] = 1U;
    labels[seed] = label;
    for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
      const std::size_t current = queue[cursor];
      const auto& point = input.points[current];

      neighbors.clear();
      const auto cell = point_cell(point, config_.cluster_eps);
      for_each_neighbor_cell(cell, [&](const Cell3& neighbor_cell) {
        const auto iterator = grid.find(neighbor_cell);
        if (iterator == grid.end()) return;
        for (const std::size_t candidate : iterator->second) {
          if (squared_distance(point, input.points[candidate]) <= radius_squared) {
            neighbors.push_back(candidate);
          }
        }
      });
      std::sort(neighbors.begin(), neighbors.end());
      neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
      for (const std::size_t neighbor : neighbors) {
        if (labels[neighbor] < 0) labels[neighbor] = label;
        if (core[neighbor] != 0U && queued_core[neighbor] == 0U) {
          queued_core[neighbor] = 1U;
          queue.push_back(neighbor);
        }
      }
    }
  }

  for (std::size_t index = 0; index < input.points.size(); ++index) {
    const int label = labels[index];
    if (label < 0) continue;
    auto& stats = clusters[static_cast<std::size_t>(label)];
    const auto& point = input.points[index];
    ++stats.count;
    stats.min_x = std::min(stats.min_x, point.x);
    stats.min_y = std::min(stats.min_y, point.y);
    stats.min_z = std::min(stats.min_z, point.z);
    stats.max_x = std::max(stats.max_x, point.x);
    stats.max_y = std::max(stats.max_y, point.y);
    stats.max_z = std::max(stats.max_z, point.z);
  }

  std::vector<unsigned char> remove_cluster(clusters.size(), 0U);
  for (std::size_t label = 0; label < clusters.size(); ++label) {
    const auto& stats = clusters[label];
    const bool small_count = stats.count <
        static_cast<std::size_t>(config_.cluster_max_points);
    const bool small_box =
        stats.max_x - stats.min_x < config_.cluster_max_bbox_x &&
        stats.max_y - stats.min_y < config_.cluster_max_bbox_y &&
        stats.max_z - stats.min_z < config_.cluster_max_bbox_z;
    if (small_count && small_box) remove_cluster[label] = 1U;
  }

  PointCloudXYZ output;
  output.points.reserve(input.points.size());
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    const int label = labels[index];
    if (label < 0 || remove_cluster[static_cast<std::size_t>(label)] == 0U) {
      output.points.push_back(input.points[index]);
    }
  }
  return output;
}

PointCloudXYZ PctMapCleaner::final_uniform_sampling(PointCloudXYZ input) const {
  if (!config_.uniform_sampling_enable || input.points.empty()) return input;

  std::unordered_set<Cell3, Cell3Hash> occupied;
  occupied.reserve(checked_reserve_size(input.points.size()));
  for (const auto& point : input.points) {
    occupied.insert(point_cell(point, config_.final_spacing));
  }
  std::vector<Cell3> cells(occupied.begin(), occupied.end());
  std::sort(cells.begin(), cells.end());

  PointCloudXYZ output;
  output.points.reserve(cells.size());
  for (const auto& cell : cells) {
    output.points.push_back(cell_center(cell, config_.final_spacing));
  }
  return output;
}

PointCloudXYZ PctMapCleaner::complete_ground_holes(PointCloudXYZ input) const {
  if (!config_.ground_completion_enable || input.points.empty()) return input;

  using SurfaceGrid = std::unordered_map<Cell2, std::vector<std::pair<double, std::size_t>>, Cell2Hash>;
  SurfaceGrid grid;
  grid.reserve(checked_reserve_size(input.points.size()));
  for (std::size_t index = 0; index < input.points.size(); ++index) {
    const auto cell = point_cell_xy(input.points[index], config_.ground_spacing);
    grid[cell].emplace_back(input.points[index].z, index);
  }

  const int radius_cells = static_cast<int>(std::ceil(
      config_.ground_support_radius / config_.ground_spacing));
  std::vector<Cell2> support_offsets;
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      const double distance = std::sqrt(
          static_cast<double>(dx * dx + dy * dy)) * config_.ground_spacing;
      if (distance <= config_.ground_support_radius + 1e-12) {
        support_offsets.push_back(Cell2{dx, dy});
      }
    }
  }

  std::unordered_set<Cell2, Cell2Hash> candidate_set;
  candidate_set.reserve(checked_reserve_size(grid.size() * 4U));
  for (const auto& item : grid) {
    for (const auto& offset : support_offsets) {
      candidate_set.insert(Cell2{item.first.x + offset.x, item.first.y + offset.y});
    }
  }
  std::vector<Cell2> candidates(candidate_set.begin(), candidate_set.end());
  std::sort(candidates.begin(), candidates.end());

  const int workers = worker_count(config_.threads);
  std::vector<std::vector<CompletionCandidate>> local_candidates(
      static_cast<std::size_t>(workers));
  const auto evaluate_candidate = [&](const Cell2& candidate,
                                      std::vector<CompletionCandidate>& output) {
    std::vector<SupportPoint> support;
    for (const auto& offset : support_offsets) {
      const Cell2 source_cell{candidate.x + offset.x, candidate.y + offset.y};
      const auto iterator = grid.find(source_cell);
      if (iterator == grid.end()) continue;
      for (const auto& point : iterator->second) {
        support.push_back(SupportPoint{
            point.first, point.second, source_cell,
            static_cast<int>(offset.x), static_cast<int>(offset.y)});
      }
    }
    if (support.empty()) return;
    std::sort(support.begin(), support.end(),
              [](const SupportPoint& left, const SupportPoint& right) {
                if (left.z != right.z) return left.z < right.z;
                return left.source_index < right.source_index;
              });

    for (std::size_t begin = 0; begin < support.size();) {
      std::size_t end = begin + 1U;
      const double cluster_min_z = support[begin].z;
      while (end < support.size() &&
             support[end].z - cluster_min_z <= config_.ground_z_tolerance) {
        ++end;
      }

      std::unordered_set<Cell2, Cell2Hash> support_cells;
      bool left = false;
      bool right = false;
      bool front = false;
      bool back = false;
      for (std::size_t index = begin; index < end; ++index) {
        support_cells.insert(support[index].source_cell);
        left = left || support[index].dx < 0;
        right = right || support[index].dx > 0;
        front = front || support[index].dy > 0;
        back = back || support[index].dy < 0;
      }
      const int direction_count = static_cast<int>(left) + static_cast<int>(right) +
          static_cast<int>(front) + static_cast<int>(back);
      if (support_cells.size() >= static_cast<std::size_t>(
              config_.ground_min_support_cells) &&
          direction_count >= config_.ground_min_support_directions) {
        const std::size_t middle_right = begin + (end - begin) / 2U;
        const std::size_t middle_left = begin + (end - begin - 1U) / 2U;
        const double estimated_z =
            (support[middle_left].z + support[middle_right].z) * 0.5;

        bool same_surface = false;
        const auto existing = grid.find(candidate);
        if (existing != grid.end()) {
          for (const auto& point : existing->second) {
            if (std::abs(point.first - estimated_z) <=
                config_.ground_z_tolerance) {
              same_surface = true;
              break;
            }
          }
        }
        if (!same_surface) {
          std::size_t nearest = begin;
          double nearest_score = std::numeric_limits<double>::infinity();
          for (std::size_t index = begin; index < end; ++index) {
            const double grid_distance = static_cast<double>(
                support[index].dx * support[index].dx +
                support[index].dy * support[index].dy);
            const double score = grid_distance +
                std::abs(support[index].z - estimated_z) * 1e-6;
            if (score < nearest_score ||
                (score == nearest_score &&
                 support[index].source_index < support[nearest].source_index)) {
              nearest = index;
              nearest_score = score;
            }
          }
          output.push_back(CompletionCandidate{
              candidate, estimated_z, support[nearest].source_index});
        }
      }
      begin = end;
    }
  };

#pragma omp parallel num_threads(workers)
  {
    const int thread =
#ifdef _OPENMP
        omp_get_thread_num();
#else
        0;
#endif
#pragma omp for schedule(dynamic, 64)
    for (std::int64_t item = 0;
         item < static_cast<std::int64_t>(candidates.size()); ++item) {
      evaluate_candidate(candidates[static_cast<std::size_t>(item)],
                         local_candidates[static_cast<std::size_t>(thread)]);
    }
  }

  std::vector<CompletionCandidate> accepted;
  for (auto& local : local_candidates) {
    accepted.insert(accepted.end(), local.begin(), local.end());
  }
  std::sort(accepted.begin(), accepted.end(),
            [](const CompletionCandidate& left, const CompletionCandidate& right) {
              if (left.cell < right.cell) return true;
              if (right.cell < left.cell) return false;
              if (left.z != right.z) return left.z < right.z;
              return left.source_index < right.source_index;
            });

  std::unordered_map<Cell2, std::vector<std::size_t>, Cell2Hash> by_cell;
  by_cell.reserve(checked_reserve_size(accepted.size()));
  for (std::size_t index = 0; index < accepted.size(); ++index) {
    by_cell[accepted[index].cell].push_back(index);
  }
  DisjointSet components(accepted.size());
  const std::array<Cell2, 8> offsets = {
      Cell2{-1, -1}, Cell2{-1, 0}, Cell2{-1, 1}, Cell2{0, -1},
      Cell2{0, 1}, Cell2{1, -1}, Cell2{1, 0}, Cell2{1, 1}};
  for (std::size_t index = 0; index < accepted.size(); ++index) {
    for (const auto& offset : offsets) {
      const Cell2 neighbor_cell{
          accepted[index].cell.x + offset.x,
          accepted[index].cell.y + offset.y};
      const auto iterator = by_cell.find(neighbor_cell);
      if (iterator == by_cell.end()) continue;
      for (const std::size_t neighbor : iterator->second) {
        if (std::abs(accepted[index].z - accepted[neighbor].z) <=
            config_.ground_z_tolerance) {
          components.unite(index, neighbor);
        }
      }
    }
  }

  const std::size_t max_hole_cells = static_cast<std::size_t>(std::floor(
      config_.ground_max_hole_area /
      (config_.ground_spacing * config_.ground_spacing) + 1e-12));
  if (max_hole_cells == 0U) {
    throw std::invalid_argument("ground_max_hole_area is smaller than one cell");
  }
  std::unordered_map<std::size_t, std::vector<std::size_t>> members;
  members.reserve(accepted.size());
  for (std::size_t index = 0; index < accepted.size(); ++index) {
    members[components.find(index)].push_back(index);
  }

  PointCloudXYZ output = std::move(input);
  output.points.reserve(output.points.size() + accepted.size());
  for (const auto& item : members) {
    if (item.second.size() > max_hole_cells) continue;
    for (const std::size_t index : item.second) {
      const auto& candidate = accepted[index];
      output.points.push_back(Point3f{
          static_cast<float>((static_cast<double>(candidate.cell.x) + 0.5) *
                             config_.ground_spacing),
          static_cast<float>((static_cast<double>(candidate.cell.y) + 0.5) *
                             config_.ground_spacing),
          static_cast<float>(candidate.z)});
    }
  }
  return output;
}

CleanerResult PctMapCleaner::clean(PointCloudXYZ input) const {
  validate();
  CleanerReport report;
  report.input_points = input.points.size();
  if (input.points.empty()) throw std::invalid_argument("input PCD is empty");

  const auto run_stage = [&](const char* name, auto&& function,
                             PointCloudXYZ& current) {
    const auto started = std::chrono::steady_clock::now();
    current = function(std::move(current));
    const auto finished = std::chrono::steady_clock::now();
    report.stage_times[name] = std::chrono::duration<double>(finished - started).count();
  };

  run_stage("finite_filter", [this](PointCloudXYZ cloud) {
    return remove_non_finite(std::move(cloud));
  }, input);
  report.finite_points = input.points.size();
  if (input.points.empty()) throw std::invalid_argument("input has no finite XYZ points");

  run_stage("pre_sampling", [this](PointCloudXYZ cloud) {
    return pre_downsample(std::move(cloud));
  }, input);
  report.pre_sampled_points = input.points.size();
  report.pre_sampling_voxels = input.points.size();

  run_stage("sor", [this](PointCloudXYZ cloud) {
    return statistical_filter(std::move(cloud));
  }, input);
  report.sor_points = input.points.size();

  run_stage("ror", [this](PointCloudXYZ cloud) {
    return radius_filter(std::move(cloud));
  }, input);
  report.ror_points = input.points.size();

  run_stage("floating_cluster", [this](PointCloudXYZ cloud) {
    return floating_cluster_filter(std::move(cloud));
  }, input);
  report.cluster_cleaned_points = input.points.size();

  run_stage("uniform_sampling", [this](PointCloudXYZ cloud) {
    return final_uniform_sampling(std::move(cloud));
  }, input);
  report.final_points = input.points.size();
  report.final_sampling_voxels = input.points.size();

  const std::size_t before_completion = input.points.size();
  run_stage("ground_completion", [this](PointCloudXYZ cloud) {
    return complete_ground_holes(std::move(cloud));
  }, input);
  report.completion_added_points = input.points.size() - before_completion;
  report.final_points = input.points.size();

  double total = 0.0;
  for (const auto& stage : report.stage_times) total += stage.second;
  report.total_time = total;
  return CleanerResult{std::move(input), std::move(report)};
}

CleanerReport PctMapCleaner::clean_file(
    const fs::path& input_path, const fs::path& output_path) const {
  PointCloudXYZ input;
  input.points = load_xyz(input_path);
  CleanerResult result = clean(std::move(input));
  write_xyz_atomic(output_path, result.cloud);
  return result.report;
}

}  // namespace reference_pct_map_cleaner
