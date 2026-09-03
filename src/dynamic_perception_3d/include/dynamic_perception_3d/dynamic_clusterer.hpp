#pragma once

#include <vector>

#include "dynamic_perception_3d/common_types.hpp"

namespace dynamic_perception_3d {

class DynamicClusterer {
 public:
  struct Config {
    double cluster_tolerance = 0.20;
    int min_cluster_size = 5;
    int max_cluster_size = 100000;
  };

  DynamicClusterer() = default;
  explicit DynamicClusterer(const Config& config);
  void SetConfig(const Config& config);
  std::vector<CloudT::Ptr> Extract(CloudT::ConstPtr cloud) const;

 private:
  Config config_;
};

}  // namespace dynamic_perception_3d
