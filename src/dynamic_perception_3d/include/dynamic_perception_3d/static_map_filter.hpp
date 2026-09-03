#pragma once

#include <string>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "dynamic_perception_3d/common_types.hpp"

namespace dynamic_perception_3d {

class StaticMapFilter {
 public:
  struct Config {
    double static_match_radius = 0.15;
    double static_recheck_radius = 0.20;
    // A PCD is a discrete sampling of static surfaces.  This secondary test
    // uses the PCD's normals, so points on the same wall do not become dynamic
    // merely because their nearest map sample is tangentially farther away.
    bool static_surface_matching_enabled = true;
    double static_surface_search_radius = 0.20;
    double static_surface_normal_distance = 0.05;
    double static_surface_min_normal_norm = 0.50;
    // Transform points from the PCD coordinate system into map_frame before
    // constructing the KD-tree. Identity keeps the original PCD coordinates.
    Eigen::Affine3d map_T_static_map = Eigen::Affine3d::Identity();
    // Optional prior for an effectively infinite horizontal static surface.
    // It is disabled by default because not every environment has one.
    bool static_ground_plane_enabled = false;
    double static_ground_plane_z = 0.0;
    double static_ground_plane_tolerance = 0.05;
  };

  struct Result {
    CloudT::Ptr static_matched{new CloudT};
    CloudT::Ptr static_surface_matched{new CloudT};
    CloudT::Ptr dynamic_candidates{new CloudT};
  };

  StaticMapFilter() = default;
  explicit StaticMapFilter(const Config& config);
  void SetConfig(const Config& config);

  bool LoadMap(const std::string& path, std::string* error = nullptr);
  Result Filter(CloudT::ConstPtr scan) const;
  double ComputeStaticRatio(CloudT::ConstPtr cluster) const;

  CloudT::ConstPtr static_map() const { return static_map_; }
  bool ready() const { return static_map_ && !static_map_->empty(); }

 private:
  enum class MatchType { kNone, kGround, kRadius, kSurface };

  bool MatchesStaticGroundPlane(const PointT& point) const;
  MatchType Classify(const PointT& point, double direct_radius) const;

  Config config_;
  CloudT::Ptr static_map_{new CloudT};
  pcl::PointCloud<pcl::PointNormal>::Ptr static_map_with_normals_{
      new pcl::PointCloud<pcl::PointNormal>};
  bool has_static_normals_ = false;
  pcl::KdTreeFLANN<PointT> static_tree_;
};

}  // namespace dynamic_perception_3d
