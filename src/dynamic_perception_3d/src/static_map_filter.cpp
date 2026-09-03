#include "dynamic_perception_3d/static_map_filter.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

namespace dynamic_perception_3d {

StaticMapFilter::StaticMapFilter(const Config& config) { SetConfig(config); }

void StaticMapFilter::SetConfig(const Config& config) {
  if (!std::isfinite(config.static_match_radius) ||
      !std::isfinite(config.static_recheck_radius) ||
      !std::isfinite(config.static_surface_search_radius) ||
      !std::isfinite(config.static_surface_normal_distance) ||
      !std::isfinite(config.static_surface_min_normal_norm) ||
      config.static_match_radius <= 0.0 ||
      config.static_recheck_radius < config.static_match_radius ||
      config.static_surface_search_radius < config.static_match_radius ||
      config.static_surface_normal_distance < 0.0 ||
      config.static_surface_min_normal_norm <= 0.0) {
    throw std::invalid_argument(
        "invalid static point/surface matching parameters");
  }
  if (!config.map_T_static_map.matrix().allFinite()) {
    throw std::invalid_argument("static_map_pose must contain finite values");
  }
  if (!std::isfinite(config.static_ground_plane_z) ||
      !std::isfinite(config.static_ground_plane_tolerance) ||
      config.static_ground_plane_tolerance < 0.0) {
    throw std::invalid_argument("invalid static ground-plane parameters");
  }
  config_ = config;
}

bool StaticMapFilter::LoadMap(const std::string& path, std::string* error) {
  if (path.empty()) {
    if (error) *error = "static_pcd_file is empty";
    return false;
  }
  pcl::PCLPointCloud2 raw_pcd;
  if (pcl::io::loadPCDFile(path, raw_pcd) < 0) {
    if (error) *error = "failed to load PCD: " + path;
    return false;
  }
  const auto field_by_name = [&raw_pcd](const std::string& name)
      -> const pcl::PCLPointField* {
    for (const pcl::PCLPointField& field : raw_pcd.fields)
      if (field.name == name) return &field;
    return nullptr;
  };
  const pcl::PCLPointField* normal_x = field_by_name("normal_x");
  const pcl::PCLPointField* normal_y = field_by_name("normal_y");
  const pcl::PCLPointField* normal_z = field_by_name("normal_z");
  has_static_normals_ = normal_x && normal_y && normal_z &&
                        normal_x->datatype == pcl::PCLPointField::FLOAT32 &&
                        normal_y->datatype == pcl::PCLPointField::FLOAT32 &&
                        normal_z->datatype == pcl::PCLPointField::FLOAT32;

  CloudT xyz_only;
  pcl::fromPCLPointCloud2(raw_pcd, xyz_only);
  pcl::PointCloud<pcl::PointNormal>::Ptr loaded(
      new pcl::PointCloud<pcl::PointNormal>);
  loaded->reserve(xyz_only.size());
  for (std::size_t index = 0; index < xyz_only.size(); ++index) {
    const PointT& point = xyz_only.points[index];
    pcl::PointNormal normal_point;
    normal_point.x = point.x;
    normal_point.y = point.y;
    normal_point.z = point.z;
    normal_point.normal_x = 0.0F;
    normal_point.normal_y = 0.0F;
    normal_point.normal_z = 0.0F;
    if (has_static_normals_) {
      const std::uint8_t* data = raw_pcd.data.data() + index * raw_pcd.point_step;
      std::memcpy(&normal_point.normal_x, data + normal_x->offset, sizeof(float));
      std::memcpy(&normal_point.normal_y, data + normal_y->offset, sizeof(float));
      std::memcpy(&normal_point.normal_z, data + normal_z->offset, sizeof(float));
    }
    loaded->push_back(normal_point);
  }
  std::vector<int> valid_indices;
  loaded->is_dense = false;
  pcl::removeNaNFromPointCloud(*loaded, *loaded, valid_indices);
  if (loaded->empty()) {
    if (error) *error = "static PCD has no valid XYZ points: " + path;
    return false;
  }
  pcl::PointCloud<pcl::PointNormal>::Ptr transformed(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::transformPointCloudWithNormals(
      *loaded, *transformed, config_.map_T_static_map.matrix().cast<float>());
  static_map_with_normals_ = transformed;
  static_map_.reset(new CloudT);
  static_map_->reserve(transformed->size());
  for (const pcl::PointNormal& point : transformed->points)
    static_map_->push_back(PointT(point.x, point.y, point.z));
  static_tree_.setInputCloud(static_map_);
  return true;
}

bool StaticMapFilter::MatchesStaticGroundPlane(const PointT& point) const {
  constexpr double kFloatComparisonTolerance = 1e-6;
  return config_.static_ground_plane_enabled &&
         std::abs(static_cast<double>(point.z) -
                  config_.static_ground_plane_z) <=
             config_.static_ground_plane_tolerance + kFloatComparisonTolerance;
}

StaticMapFilter::MatchType StaticMapFilter::Classify(
    const PointT& point, double direct_radius) const {
  if (MatchesStaticGroundPlane(point)) return MatchType::kGround;
  std::vector<int> indices;
  std::vector<float> squared_distances;
  if (static_tree_.radiusSearch(point, direct_radius, indices,
                                squared_distances, 1) > 0) {
    return MatchType::kRadius;
  }
  if (!config_.static_surface_matching_enabled || !has_static_normals_)
    return MatchType::kNone;
  indices.clear();
  squared_distances.clear();
  if (static_tree_.radiusSearch(point, config_.static_surface_search_radius,
                                indices, squared_distances) == 0) {
    return MatchType::kNone;
  }
  for (const int index : indices) {
    const pcl::PointNormal& map_point = static_map_with_normals_->points[index];
    const Eigen::Vector3d normal(map_point.normal_x, map_point.normal_y,
                                 map_point.normal_z);
    const double normal_norm = normal.norm();
    if (!std::isfinite(normal_norm) ||
        normal_norm < config_.static_surface_min_normal_norm) {
      continue;
    }
    const Eigen::Vector3d residual(
        static_cast<double>(point.x - map_point.x),
        static_cast<double>(point.y - map_point.y),
        static_cast<double>(point.z - map_point.z));
    if (std::abs(residual.dot(normal / normal_norm)) <=
        config_.static_surface_normal_distance) {
      return MatchType::kSurface;
    }
  }
  return MatchType::kNone;
}

StaticMapFilter::Result StaticMapFilter::Filter(CloudT::ConstPtr scan) const {
  Result result;
  if (!scan || scan->empty() || !ready()) return result;
  result.static_matched->reserve(scan->size());
  result.static_surface_matched->reserve(scan->size());
  result.dynamic_candidates->reserve(scan->size());
  for (const PointT& point : scan->points) {
    const MatchType match = Classify(point, config_.static_match_radius);
    if (match != MatchType::kNone) {
      result.static_matched->push_back(point);
      if (match == MatchType::kSurface)
        result.static_surface_matched->push_back(point);
    } else {
      result.dynamic_candidates->push_back(point);
    }
  }
  result.static_matched->width = result.static_matched->size();
  result.static_matched->height = 1;
  result.static_surface_matched->width = result.static_surface_matched->size();
  result.static_surface_matched->height = 1;
  result.dynamic_candidates->width = result.dynamic_candidates->size();
  result.dynamic_candidates->height = 1;
  return result;
}

double StaticMapFilter::ComputeStaticRatio(CloudT::ConstPtr cluster) const {
  if (!cluster || cluster->empty() || !ready()) return 0.0;
  std::size_t hits = 0;
  for (const PointT& point : cluster->points) {
    if (Classify(point, config_.static_recheck_radius) != MatchType::kNone) {
      ++hits;
    }
  }
  return static_cast<double>(hits) / static_cast<double>(cluster->size());
}

}  // namespace dynamic_perception_3d
