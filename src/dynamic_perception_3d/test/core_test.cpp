#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

#include "dynamic_perception_3d/dynamic_clusterer.hpp"
#include "dynamic_perception_3d/obstacle_manager.hpp"
#include "dynamic_perception_3d/static_map_filter.hpp"
#include "dynamic_perception_3d/visibility_clearer.hpp"

namespace dp = dynamic_perception_3d;

namespace {
dp::CloudT::Ptr MakeCloud(const std::vector<Eigen::Vector3d>& points) {
  dp::CloudT::Ptr cloud(new dp::CloudT);
  for (const Eigen::Vector3d& point : points) {
    cloud->push_back(dp::PointT(point.x(), point.y(), point.z()));
  }
  return cloud;
}

dp::CloudT::Ptr MakeCluster(double x, double y = 0.0, double z = 0.0) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 6; ++i) points.emplace_back(x, y + 0.02 * i, z);
  return MakeCloud(points);
}
}  // namespace

TEST(StaticMapFilter, PointLevelSubtractionAndRelaxedRecheck) {
  dp::StaticMapFilter::Config config;
  config.static_match_radius = 0.05;
  config.static_recheck_radius = 0.15;
  dp::StaticMapFilter filter(config);
  std::string error;
  ASSERT_TRUE(filter.LoadMap(
      std::string(DYNAMIC_PERCEPTION_TEST_DATA_DIR) + "/synthetic_static_map.pcd",
      &error)) << error;
  dp::CloudT::Ptr scan = MakeCloud({{2.01, 0.0, 0.0}, {2.10, 0.2, 0.0},
                                    {1.0, 1.0, 0.5}});
  const auto result = filter.Filter(scan);
  EXPECT_EQ(1u, result.static_matched->size());
  EXPECT_EQ(2u, result.dynamic_candidates->size());
  EXPECT_NEAR(0.5, filter.ComputeStaticRatio(result.dynamic_candidates), 1e-6);
}

TEST(StaticMapFilter, AppliesMapPoseAndOptionalGroundPlane) {
  dp::StaticMapFilter::Config config;
  config.static_match_radius = 0.03;
  config.static_recheck_radius = 0.05;
  config.map_T_static_map = Eigen::Translation3d(0.0, 0.0, -0.1);
  config.static_ground_plane_enabled = true;
  config.static_ground_plane_z = 0.0;
  config.static_ground_plane_tolerance = 0.05;
  dp::StaticMapFilter filter(config);
  std::string error;
  ASSERT_TRUE(filter.LoadMap(
      std::string(DYNAMIC_PERCEPTION_TEST_DATA_DIR) + "/synthetic_static_map.pcd",
      &error)) << error;

  ASSERT_FALSE(filter.static_map()->empty());
  EXPECT_NEAR(-0.1, filter.static_map()->front().z, 1e-6);
  dp::CloudT::Ptr scan = MakeCloud({{2.0, 0.0, -0.1},
                                    {100.0, 100.0, 0.04},
                                    {100.0, 100.0, 0.20}});
  const auto result = filter.Filter(scan);
  EXPECT_EQ(2u, result.static_matched->size());
  ASSERT_EQ(1u, result.dynamic_candidates->size());

  dp::CloudT::Ptr recheck =
      MakeCloud({{100.0, 100.0, -0.05}, {100.0, 100.0, 0.20}});
  EXPECT_NEAR(0.5, filter.ComputeStaticRatio(recheck), 1e-6);
}

TEST(StaticMapFilter, MatchesStaticWallByPcdNormalButKeepsProtrusion) {
  dp::StaticMapFilter::Config config;
  config.static_match_radius = 0.04;
  config.static_recheck_radius = 0.04;
  config.static_surface_search_radius = 0.20;
  config.static_surface_normal_distance = 0.05;
  dp::StaticMapFilter filter(config);
  std::string error;
  ASSERT_TRUE(filter.LoadMap(
      std::string(DYNAMIC_PERCEPTION_TEST_DATA_DIR) +
          "/synthetic_wall_with_normals.pcd",
      &error)) << error;

  // The first point lies on the mapped x=0 wall but between sparse PCD
  // samples. The second is a box face that protrudes 12 cm from that wall.
  dp::CloudT::Ptr scan = MakeCloud({{0.03, 0.18, 0.5}, {0.12, 0.18, 0.5}});
  const auto result = filter.Filter(scan);
  EXPECT_EQ(1u, result.static_matched->size());
  EXPECT_EQ(1u, result.static_surface_matched->size());
  ASSERT_EQ(1u, result.dynamic_candidates->size());
  EXPECT_NEAR(0.12, result.dynamic_candidates->front().x, 1e-6);
}

TEST(StaticMapFilter, RejectsInvalidAlignmentParameters) {
  dp::StaticMapFilter::Config config;
  config.map_T_static_map.matrix()(0, 0) =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dp::StaticMapFilter filter(config), std::invalid_argument);

  config = dp::StaticMapFilter::Config();
  config.static_ground_plane_tolerance = -0.01;
  EXPECT_THROW(dp::StaticMapFilter filter(config), std::invalid_argument);
}

TEST(DynamicClusterer, SeparatesClustersAndRejectsNoise) {
  dp::DynamicClusterer::Config config;
  config.cluster_tolerance = 0.15;
  config.min_cluster_size = 3;
  dp::DynamicClusterer clusterer(config);
  dp::CloudT::Ptr cloud = MakeCloud({{1.0, 0.0, 0.0}, {1.0, 0.05, 0.0},
                                     {1.0, 0.10, 0.0}, {3.0, 0.0, 0.0},
                                     {3.0, 0.05, 0.0}, {3.0, 0.10, 0.0},
                                     {8.0, 8.0, 8.0}});
  const auto clusters = clusterer.Extract(cloud);
  ASSERT_EQ(2u, clusters.size());
  EXPECT_EQ(3u, clusters[0]->size());
  EXPECT_EQ(3u, clusters[1]->size());
}

TEST(VisibilityClearer, HandlesFovOcclusionAndObservation) {
  dp::VisibilityClearer::Config config;
  config.min_range = 0.0;
  config.max_range = 10.0;
  config.vertical_fov_bottom_deg = -45.0;
  config.vertical_fov_top_deg = 45.0;
  config.ray_step = 0.1;
  config.ray_search_radius = 0.08;
  config.ray_target_guard = 0.2;
  dp::VisibilityClearer clearer(config);
  const Eigen::Vector3d point(2.0, 0.05, 0.0);
  dp::SensorPose pose;
  EXPECT_TRUE(clearer.IsPointInsideFov(point, pose));

  dp::CloudT::Ptr scan = MakeCloud({{1.0, 0.025, 0.0}, {2.0, 0.05, 0.0}});
  pcl::KdTreeFLANN<dp::PointT> tree;
  tree.setInputCloud(scan);
  EXPECT_TRUE(clearer.IsPointOccluded(point, pose, tree));
  EXPECT_TRUE(clearer.IsPointObservedInScan(point, tree));

  dp::CloudT::Ptr target_only = MakeCloud({{2.0, 0.05, 0.0}});
  tree.setInputCloud(target_only);
  EXPECT_FALSE(clearer.IsPointOccluded(point, pose, tree));
  EXPECT_TRUE(clearer.IsPointObservedInScan(point, tree));

  dp::CloudT::Ptr behind_target = MakeCloud({{2.5, 0.0625, 0.0}});
  tree.setInputCloud(behind_target);
  EXPECT_FALSE(clearer.IsPointObservedInScan(point, tree));

  dp::CloudT::Ptr unrelated = MakeCloud({{2.0, 2.0, 0.0}});
  tree.setInputCloud(unrelated);
  EXPECT_FALSE(clearer.IsPointObservedInScan(point, tree));
}

TEST(VisibilityClearer, EvaluatesEachVoxelIndependentlyInFov) {
  dp::VisibilityClearer::Config config;
  config.min_range = 0.0;
  config.max_range = 10.0;
  config.horizontal_fov_min_deg = -10.0;
  config.horizontal_fov_max_deg = 10.0;
  config.vertical_fov_bottom_deg = -45.0;
  config.vertical_fov_top_deg = 45.0;
  dp::VisibilityClearer clearer(config);
  dp::SensorPose pose;

  EXPECT_TRUE(clearer.IsPointInsideFov(Eigen::Vector3d(2.0, 0.1, 0.0), pose));
  EXPECT_FALSE(clearer.IsPointInsideFov(Eigen::Vector3d(2.0, 1.0, 0.0), pose));
}

TEST(ObstacleManager, FusesStationaryViewsWithoutShrinkingOutput) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.association_distance = 1.5;
  manager_config.obstacle_voxel_size = 0.01;
  manager_config.confirm_hits = 1;
  manager_config.history_window_size = 200.0;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr front = MakeCluster(2.0, 0.0, 0.0);
  // The visible-surface centroid moves by 1 m when observing opposite faces
  // of this large static obstacle. A fixed centroid-reset threshold would
  // incorrectly discard the first face.
  dp::CloudT::Ptr back = MakeCluster(3.0, 0.0, 0.0);

  manager.Update({front}, front, front, pose, 1.0);
  ASSERT_EQ(front->size(), manager.GetOutputCloud()->size());
  const uint64_t id = manager.GetObstacles().front().id;
  dp::SensorPose opposite_pose;
  opposite_pose.map_T_sensor.translation().x() = 5.0;
  manager.Update({back}, back, back, opposite_pose, 2.0);

  const auto obstacles = manager.GetObstacles();
  ASSERT_EQ(1u, obstacles.size());
  EXPECT_EQ(id, obstacles.front().id);
  EXPECT_EQ(front->size() + back->size(), manager.GetOutputCloud()->size());
  EXPECT_NEAR(3.0, obstacles.front().observation_centroid.x(), 1e-6);
  EXPECT_LT(obstacles.front().min_bound.x(), 2.1);
  EXPECT_GT(obstacles.front().max_bound.x(), 2.9);

  dp::SensorPose out_of_view_pose;
  out_of_view_pose.map_T_sensor.translation().x() = 100.0;
  out_of_view_pose.map_T_base.translation().x() = 100.0;
  dp::CloudT::Ptr no_dynamic_points(new dp::CloudT);
  manager.Update({}, front, no_dynamic_points, out_of_view_pose, 3.0);
  const auto occluded = manager.GetObstacles();
  ASSERT_EQ(1u, occluded.size());
  EXPECT_EQ(dp::ObstacleState::OCCLUDED, occluded.front().state);
  EXPECT_EQ(front->size() + back->size(), manager.GetOutputCloud()->size());
}

TEST(ObstacleManager, ClearsOldVoxelsAfterMovingObstacleLeavesThem) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.association_distance = 1.0;
  manager_config.obstacle_voxel_size = 0.01;
  manager_config.confirm_hits = 1;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr initial = MakeCluster(2.0);
  dp::CloudT::Ptr moved = MakeCluster(2.8);

  manager.Update({initial}, initial, initial, pose, 1.0);
  manager.Update({moved}, moved, moved, pose, 2.0);
  const auto obstacles = manager.GetObstacles();
  ASSERT_EQ(1u, obstacles.size());
  EXPECT_EQ(moved->size(), manager.GetOutputCloud()->size());
  EXPECT_GT(obstacles.front().min_bound.x(), 2.7);
}

TEST(ObstacleManager, ClearsConfirmedObstacleWhenTargetHasNoReturn) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 1;
  manager_config.obstacle_voxel_size = 0.01;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr initial = MakeCluster(2.0);
  dp::CloudT::Ptr no_dynamic_points(new dp::CloudT);
  // These returns are in the sensor FOV but neither occlude nor pass through
  // the original obstacle's ray corridor.
  dp::CloudT::Ptr unrelated = MakeCloud({{2.0, 2.0, 0.0}});

  manager.Update({initial}, initial, initial, pose, 1.0);
  manager.Update({}, unrelated, no_dynamic_points, pose, 2.0);
  EXPECT_TRUE(manager.GetObstacles().empty());
  EXPECT_TRUE(manager.GetOutputCloud()->empty());
}

TEST(ObstacleManager, RetainsObstacleWhenTargetNeighborhoodHasReturn) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 1;
  manager_config.obstacle_voxel_size = 0.01;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr initial = MakeCluster(2.0);
  manager.Update({initial}, initial, initial, pose, 1.0);
  ASSERT_FALSE(manager.GetObstacles().empty());

  manager.Update({}, initial, dp::CloudT::Ptr(new dp::CloudT), pose,
                 2.0);
  EXPECT_FALSE(manager.GetObstacles().empty());
}

TEST(ObstacleManager, RejectsInvalidVoxelConfiguration) {
  dp::ObstacleManager::Config config;
  config.obstacle_voxel_size = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
      dp::ObstacleManager manager(config, dp::VisibilityClearer()),
      std::invalid_argument);
}

TEST(ObstacleManager, RemovesHistoryOutsideLocalWindow) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 1;
  manager_config.history_window_size = 1.0;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr cluster = MakeCluster(0.5);
  manager.Update({cluster}, cluster, cluster, pose, 1.0);
  ASSERT_FALSE(manager.GetOutputCloud()->empty());

  pose.map_T_base.translation().x() = 3.0;
  pose.map_T_sensor.translation().x() = 3.0;
  manager.Update({}, MakeCluster(3.0), dp::CloudT::Ptr(new dp::CloudT), pose,
                 2.0);
  EXPECT_TRUE(manager.GetObstacles().empty());
  EXPECT_TRUE(manager.GetOutputCloud()->empty());
}

TEST(ObstacleManager, RemovesHistoryOutsideIndependentHeightWindow) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 1;
  manager_config.history_window_size = 2.0;
  manager_config.history_min_height = -0.25;
  manager_config.history_max_height = 0.25;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr cluster = MakeCluster(0.5, 0.0, 0.2);
  manager.Update({cluster}, cluster, cluster, pose, 1.0);
  ASSERT_FALSE(manager.GetOutputCloud()->empty());

  pose.map_T_base.translation().z() = 1.0;
  manager.Update({}, dp::CloudT::Ptr(new dp::CloudT),
                 dp::CloudT::Ptr(new dp::CloudT), pose, 2.0);
  EXPECT_TRUE(manager.GetObstacles().empty());
  EXPECT_TRUE(manager.GetOutputCloud()->empty());
}

TEST(ObstacleManager, RotatedBaseWindowRetainsLocalHistoryVoxels) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 1;
  manager_config.history_window_size = 1.0;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  pose.map_T_base =
      Eigen::AngleAxisd(std::acos(-1.0) / 4.0, Eigen::Vector3d::UnitZ());
  dp::CloudT::Ptr cluster = MakeCluster(1.2);
  manager.Update({cluster}, cluster, cluster, pose, 1.0);
  ASSERT_FALSE(manager.GetObstacles().empty());

  manager.Update({}, dp::CloudT::Ptr(new dp::CloudT),
                 dp::CloudT::Ptr(new dp::CloudT), pose, 2.0);
  EXPECT_FALSE(manager.GetObstacles().empty());

  pose.map_T_base = Eigen::Affine3d::Identity();
  manager.Update({}, dp::CloudT::Ptr(new dp::CloudT),
                 dp::CloudT::Ptr(new dp::CloudT), pose, 3.0);
  EXPECT_TRUE(manager.GetObstacles().empty());
}

TEST(ObstacleManager, UsesVisibilityReturnOutsideHistoryWindowForClearing) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  visibility_config.ray_step = 0.05;
  visibility_config.ray_search_radius = 0.05;
  visibility_config.ray_target_guard = 0.10;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 1;
  manager_config.obstacle_voxel_size = 0.01;
  manager_config.history_window_size = 1.0;
  dp::ObstacleManager manager(manager_config,
                              dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr initial = MakeCluster(0.9);
  manager.Update({initial}, initial, initial, pose, 1.0);
  ASSERT_FALSE(manager.GetObstacles().empty());

  // The target is inside the 1 m base window. The unrelated return is outside
  // the target neighborhood and therefore triggers DDDMR-style direct clear.
  dp::CloudT::Ptr behind_target = MakeCluster(1.2);
  manager.Update({}, behind_target, dp::CloudT::Ptr(new dp::CloudT), pose, 2.0);
  EXPECT_TRUE(manager.GetObstacles().empty());
}

TEST(ObstacleManager, ConfirmsClearsOccludesAndRetainsConfirmedObstacles) {
  dp::VisibilityClearer::Config visibility_config;
  visibility_config.min_range = 0.0;
  visibility_config.max_range = 10.0;
  visibility_config.vertical_fov_bottom_deg = -45.0;
  visibility_config.vertical_fov_top_deg = 45.0;
  visibility_config.ray_step = 0.1;
  visibility_config.ray_search_radius = 0.08;
  dp::ObstacleManager::Config manager_config;
  manager_config.confirm_hits = 2;
  manager_config.max_tentative_age = 5.0;
  dp::ObstacleManager manager(manager_config, dp::VisibilityClearer(visibility_config));
  dp::SensorPose pose;
  dp::CloudT::Ptr cluster = MakeCluster(2.0);
  dp::CloudT::Ptr observed(new dp::CloudT(*cluster));

  manager.Update({cluster}, observed, observed, pose, 1.0);
  EXPECT_TRUE(manager.GetOutputCloud()->empty());
  const uint64_t id = manager.GetObstacles().front().id;
  manager.Update({cluster}, observed, observed, pose, 2.0);
  ASSERT_FALSE(manager.GetOutputCloud()->empty());
  EXPECT_EQ(id, manager.GetObstacles().front().id);

  dp::CloudT::Ptr occluding = MakeCloud({{1.0, 0.05, 0.0}});
  dp::CloudT::Ptr no_dynamic_points(new dp::CloudT);
  manager.Update({}, occluding, no_dynamic_points, pose, 3.0);
  EXPECT_EQ(dp::ObstacleState::OCCLUDED, manager.GetObstacles().front().state);
  EXPECT_FALSE(manager.GetOutputCloud()->empty());

  // A return outside the old target neighborhood triggers direct clear.
  dp::CloudT::Ptr static_only_scan = MakeCluster(2.6);
  manager.Update({}, static_only_scan, no_dynamic_points, pose, 4.0);
  EXPECT_TRUE(manager.GetObstacles().empty());

  // A confirmed obstacle outside the sensing range is retained indefinitely;
  // time alone must never release published obstacle space.
  manager.Update({cluster}, observed, observed, pose, 10.0);
  manager.Update({cluster}, observed, observed, pose, 10.1);
  manager.Update({}, occluding, no_dynamic_points, pose, 100.0);
  ASSERT_EQ(1u, manager.GetObstacles().size());
  EXPECT_EQ(dp::ObstacleState::OCCLUDED, manager.GetObstacles().front().state);
  EXPECT_FALSE(manager.GetOutputCloud()->empty());

  dp::SensorPose out_of_view_pose;
  out_of_view_pose.map_T_sensor.translation().x() = 100.0;
  out_of_view_pose.map_T_base.translation().x() = 100.0;
  manager.Update({}, observed, no_dynamic_points, out_of_view_pose, 101.0);
  EXPECT_TRUE(manager.GetObstacles().empty());

  // A stale TENTATIVE is safe to prune because it is never published.
  dp::ObstacleManager::Config tentative_config = manager_config;
  tentative_config.max_tentative_age = 0.5;
  dp::ObstacleManager tentative_manager(
      tentative_config, dp::VisibilityClearer(visibility_config));
  tentative_manager.Update({cluster}, observed, observed, pose, 1.0);
  ASSERT_EQ(1u, tentative_manager.GetObstacles().size());
  tentative_manager.Update({}, observed, no_dynamic_points, out_of_view_pose, 2.0);
  EXPECT_TRUE(tentative_manager.GetObstacles().empty());
}
