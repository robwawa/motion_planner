#include "dynamic_perception_3d/dynamic_clusterer.hpp"

#include <stdexcept>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace dynamic_perception_3d {

DynamicClusterer::DynamicClusterer(const Config& config) { SetConfig(config); }

void DynamicClusterer::SetConfig(const Config& config) {
  if (config.cluster_tolerance <= 0.0 || config.min_cluster_size <= 0 ||
      config.max_cluster_size < config.min_cluster_size) {
    throw std::invalid_argument("invalid Euclidean clustering configuration");
  }
  config_ = config;
}

std::vector<CloudT::Ptr> DynamicClusterer::Extract(CloudT::ConstPtr cloud) const {
  std::vector<CloudT::Ptr> result;
  if (!cloud || cloud->empty()) return result;

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<PointT> extraction;
  extraction.setClusterTolerance(config_.cluster_tolerance);
  extraction.setMinClusterSize(config_.min_cluster_size);
  extraction.setMaxClusterSize(config_.max_cluster_size);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  extraction.extract(cluster_indices);

  result.reserve(cluster_indices.size());
  for (const pcl::PointIndices& indices : cluster_indices) {
    CloudT::Ptr cluster(new CloudT);
    cluster->reserve(indices.indices.size());
    for (int index : indices.indices) cluster->push_back((*cloud)[index]);
    cluster->width = cluster->size();
    cluster->height = 1;
    result.push_back(cluster);
  }
  return result;
}

}  // namespace dynamic_perception_3d
