#pragma once

#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace pct_map_cleaner {

struct Point3f {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct PointCloudXYZ {
  std::vector<Point3f> points;
};

enum class SorSearchBackend { Legacy, SingleExact };

struct CleanerConfig {
  std::string input_pcd;
  std::string output_pcd;

  bool pre_sampling_enable = true;
  double pre_spacing = 0.025;
  double max_pre_rep_error = 0.05;

  bool sor_enable = true;
  SorSearchBackend sor_search_backend = SorSearchBackend::Legacy;
  int sor_neighbors = 16;
  double sor_std_ratio = 2.5;

  bool ror_enable = true;
  int ror_min_neighbors = 2;
  double ror_radius = 0.10;

  bool floating_cluster_enable = true;
  double cluster_eps = 0.15;
  int cluster_min_points = 3;
  int cluster_max_points = 10;
  double cluster_max_bbox_x = 0.30;
  double cluster_max_bbox_y = 0.30;
  double cluster_max_bbox_z = 0.30;

  bool uniform_sampling_enable = true;
  double final_spacing = 0.05;

  bool ground_completion_enable = true;
  double ground_spacing = 0.05;
  double ground_support_radius = 0.30;
  double ground_z_tolerance = 0.15;
  int ground_min_support_directions = 4;
  int ground_min_support_cells = 4;
  double ground_max_hole_area = 0.25;

  int threads = 0;
  bool save_intermediate = false;
};

struct CleanerReport {
  std::size_t input_points = 0;
  std::size_t finite_points = 0;
  std::size_t pre_sampled_points = 0;
  std::size_t sor_points = 0;
  std::size_t ror_points = 0;
  std::size_t cluster_cleaned_points = 0;
  std::size_t final_points = 0;
  std::size_t completion_added_points = 0;
  std::size_t pre_sampling_voxels = 0;
  std::size_t final_sampling_voxels = 0;
  std::size_t completion_candidate_cells = 0;
  std::size_t completion_holes_filled = 0;

  std::map<std::string, double> stage_times;
  std::map<std::string, double> detail_times;
  double total_time = 0.0;
};

struct CleanerResult {
  PointCloudXYZ cloud;
  CleanerReport report;
};

class PctMapCleaner final {
public:
  explicit PctMapCleaner(CleanerConfig config);

  void validate() const;

  CleanerResult clean(PointCloudXYZ input) const;

  CleanerReport clean_file(
      const std::filesystem::path& input_path,
      const std::filesystem::path& output_path) const;

  const CleanerConfig& config() const noexcept { return config_; }

private:
  CleanerConfig config_;

  PointCloudXYZ remove_non_finite(PointCloudXYZ input) const;
  PointCloudXYZ pre_downsample(PointCloudXYZ input,
      std::map<std::string, double>& detail_times) const;
  PointCloudXYZ statistical_filter(PointCloudXYZ input,
      std::map<std::string, double>& detail_times) const;
  PointCloudXYZ radius_filter(PointCloudXYZ input,
      std::map<std::string, double>& detail_times) const;
  PointCloudXYZ floating_cluster_filter(PointCloudXYZ input) const;
  PointCloudXYZ final_uniform_sampling(PointCloudXYZ input) const;
  PointCloudXYZ complete_ground_holes(PointCloudXYZ input,
      std::map<std::string, double>& detail_times) const;
};

}  // namespace pct_map_cleaner
