#include "pct_map_cleaner.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#ifndef PCT_MAP_CLEANER_DEFAULT_CONFIG
#define PCT_MAP_CLEANER_DEFAULT_CONFIG ""
#endif

namespace fs = std::filesystem;
using pct_map_cleaner::CleanerConfig;
using pct_map_cleaner::CleanerReport;
using pct_map_cleaner::PctMapCleaner;

namespace {

struct Arguments {
  fs::path config;
  std::string input_override;
  std::string output_override;
  bool help = false;
};

void print_help(const char* executable) {
  std::cout
      << "Usage: " << executable
      << " [--config FILE] [--input PCD] [--output PCD]\n\n"
      << "Independent C++ PCT point-cloud cleaner.\n\n"
      << "Options:\n"
      << "  --config FILE  YAML configuration. Defaults to the bundled config.\n"
      << "  --input PCD    Override input_pcd from YAML.\n"
      << "  --output PCD   Override output_pcd from YAML.\n"
      << "  --help         Show this help.\n";
}

Arguments parse_arguments(int argc, char** argv) {
  Arguments arguments;
#ifdef PCT_MAP_CLEANER_DEFAULT_CONFIG
  arguments.config = PCT_MAP_CLEANER_DEFAULT_CONFIG;
#endif
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      arguments.help = true;
    } else if (argument == "--config" && index + 1 < argc) {
      arguments.config = argv[++index];
    } else if (argument == "--input" && index + 1 < argc) {
      arguments.input_override = argv[++index];
    } else if (argument == "--output" && index + 1 < argc) {
      arguments.output_override = argv[++index];
    } else {
      throw std::invalid_argument("unknown or incomplete argument: " + argument);
    }
  }
  return arguments;
}

template <typename T>
void read_value(const YAML::Node& root, const char* section, const char* key, T& target) {
  const YAML::Node section_node = root[section];
  if (section_node && section_node[key]) target = section_node[key].as<T>();
}

fs::path resolve_config_path(const fs::path& config_path, const std::string& value) {
  const fs::path candidate(value);
  if (candidate.is_absolute()) return candidate;
  return fs::absolute(config_path.parent_path() / candidate).lexically_normal();
}

CleanerConfig load_config(const fs::path& path) {
  if (path.empty()) {
    throw std::runtime_error("no configuration file was specified");
  }
  const fs::path config_path = fs::absolute(path).lexically_normal();
  if (!fs::is_regular_file(config_path)) {
    throw std::runtime_error("configuration file does not exist: " +
                             config_path.string());
  }
  YAML::Node root = YAML::LoadFile(config_path.string());
  if (!root || !root.IsMap()) {
    throw std::runtime_error("configuration root must be a YAML mapping");
  }

  CleanerConfig config;
  if (root["input_pcd"]) {
    config.input_pcd = resolve_config_path(
        config_path, root["input_pcd"].as<std::string>()).string();
  }
  if (root["output_pcd"]) {
    config.output_pcd = resolve_config_path(
        config_path, root["output_pcd"].as<std::string>()).string();
  }

  read_value(root, "pre_sampling", "enable", config.pre_sampling_enable);
  read_value(root, "pre_sampling", "spacing", config.pre_spacing);
  read_value(root, "pre_sampling", "max_rep_error", config.max_pre_rep_error);
  if (root["pre_sampling"] && root["pre_sampling"]["representative"] &&
      root["pre_sampling"]["representative"].as<std::string>() !=
          "nearest_to_voxel_center") {
    throw std::invalid_argument(
        "pre_sampling.representative must be nearest_to_voxel_center");
  }

  read_value(root, "sor", "enable", config.sor_enable);
  std::string backend = "legacy";
  read_value(root, "sor", "search_backend", backend);
  if (backend == "legacy") {
    config.sor_search_backend = pct_map_cleaner::SorSearchBackend::Legacy;
  } else if (backend == "single_exact") {
    config.sor_search_backend = pct_map_cleaner::SorSearchBackend::SingleExact;
  } else {
    throw std::invalid_argument("sor.search_backend must be legacy or single_exact");
  }
  read_value(root, "sor", "nb_neighbors", config.sor_neighbors);
  read_value(root, "sor", "std_ratio", config.sor_std_ratio);

  read_value(root, "ror", "enable", config.ror_enable);
  read_value(root, "ror", "nb_points", config.ror_min_neighbors);
  read_value(root, "ror", "radius", config.ror_radius);

  read_value(root, "floating_cluster", "enable", config.floating_cluster_enable);
  read_value(root, "floating_cluster", "eps", config.cluster_eps);
  read_value(root, "floating_cluster", "min_points", config.cluster_min_points);
  read_value(root, "floating_cluster", "remove_if_point_count_below",
             config.cluster_max_points);
  read_value(root, "floating_cluster", "max_bbox_x", config.cluster_max_bbox_x);
  read_value(root, "floating_cluster", "max_bbox_y", config.cluster_max_bbox_y);
  read_value(root, "floating_cluster", "max_bbox_z", config.cluster_max_bbox_z);

  read_value(root, "uniform_sampling", "enable", config.uniform_sampling_enable);
  read_value(root, "uniform_sampling", "spacing", config.final_spacing);
  if (root["uniform_sampling"] && root["uniform_sampling"]["sampling_mode"] &&
      root["uniform_sampling"]["sampling_mode"].as<std::string>() !=
          "voxel_center") {
    throw std::invalid_argument(
        "uniform_sampling.sampling_mode must be voxel_center");
  }

  read_value(root, "ground_completion", "enable", config.ground_completion_enable);
  read_value(root, "ground_completion", "spacing", config.ground_spacing);
  read_value(root, "ground_completion", "support_radius", config.ground_support_radius);
  read_value(root, "ground_completion", "z_tolerance", config.ground_z_tolerance);
  read_value(root, "ground_completion", "min_support_directions",
             config.ground_min_support_directions);
  read_value(root, "ground_completion", "min_support_cells",
             config.ground_min_support_cells);
  read_value(root, "ground_completion", "max_hole_area",
             config.ground_max_hole_area);

  read_value(root, "performance", "threads", config.threads);
  read_value(root, "debug", "save_intermediate", config.save_intermediate);
  return config;
}

void print_report(const CleanerReport& report, const fs::path& output) {
  std::cout << "========== PCT Map Cleaner C++ Report ==========" << std::endl
            << "  input points: " << report.input_points << std::endl
            << "  finite points: " << report.finite_points << std::endl
            << "  pre-sampled points: " << report.pre_sampled_points << std::endl
            << "  after SOR: " << report.sor_points << std::endl
            << "  after ROR: " << report.ror_points << std::endl
            << "  after cluster filter: " << report.cluster_cleaned_points << std::endl
            << "  final points: " << report.final_points << std::endl
            << "  completion added: " << report.completion_added_points << std::endl;
  for (const auto& stage : report.stage_times) {
    std::cout << "  " << stage.first << ": " << stage.second << " s" << std::endl;
  }
  for (const auto& detail : report.detail_times) {
    std::cout << "  detail." << detail.first << ": " << detail.second << " s" << std::endl;
  }
  std::cout << "  total: " << report.total_time << " s" << std::endl
            << "  output: " << output << std::endl
            << "===============================================" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments arguments = parse_arguments(argc, argv);
    if (arguments.help) {
      print_help(argv[0]);
      return 0;
    }

    CleanerConfig config = load_config(arguments.config);
    if (!arguments.input_override.empty()) config.input_pcd = arguments.input_override;
    if (!arguments.output_override.empty()) config.output_pcd = arguments.output_override;
    if (config.input_pcd.empty()) {
      throw std::runtime_error("input_pcd is missing from YAML and --input was not provided");
    }
    if (config.output_pcd.empty()) {
      throw std::runtime_error("output_pcd is missing from YAML and --output was not provided");
    }

    PctMapCleaner cleaner(config);
    const CleanerReport report = cleaner.clean_file(config.input_pcd, config.output_pcd);
    print_report(report, config.output_pcd);
    std::cout << "  sor backend: " << (config.sor_enable ?
        (config.sor_search_backend == pct_map_cleaner::SorSearchBackend::Legacy ?
         "legacy" : "single_exact") : "disabled") << std::endl
              << "  requested threads: " << config.threads << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "[ERROR] " << error.what() << std::endl;
    return 1;
  }
}
