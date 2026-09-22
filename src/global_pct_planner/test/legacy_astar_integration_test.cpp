#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "a_star/a_star_search.h"
#include "global_pct_planner/dynamic_obstacle_layer.h"
#include "global_pct_planner/planner_core.h"

namespace global_pct_planner {
namespace {

TomogramData testMap() {
  TomogramData map;
  map.resolution = 1.0f;
  map.center_x = 0.0f;
  map.center_y = 0.0f;
  map.slice_h0 = 0.0f;
  map.slice_dh = 1.0f;
  map.layers = 1;
  map.rows = 15;
  map.cols = 15;
  const std::size_t count = map.cellCount();
  map.traversability.assign(count, 0.0f);
  map.traversability_grad_x.assign(count, 0.0f);
  map.traversability_grad_y.assign(count, 0.0f);
  map.ground_elevation.assign(count, 0.0f);
  map.ceiling_elevation.assign(count, 2.0f);
  return map;
}

struct LegacyMap {
  Eigen::MatrixXd cost;
  Eigen::MatrixXd height;
  Eigen::MatrixXd gateway;
};

LegacyMap legacyMap(const TomogramData& map) {
  LegacyMap result;
  result.cost = Eigen::MatrixXd::Zero(
      static_cast<int>(map.layers * map.rows), static_cast<int>(map.cols));
  result.height = Eigen::MatrixXd::Zero(result.cost.rows(), result.cost.cols());
  result.gateway = Eigen::MatrixXd::Zero(result.cost.rows(), result.cost.cols());
  for (uint32_t layer = 0; layer < map.layers; ++layer) {
    for (uint32_t row = 0; row < map.rows; ++row) {
      for (uint32_t col = 0; col < map.cols; ++col) {
        const std::size_t id = map.index(layer, row, col);
        const int matrix_row = static_cast<int>(layer * map.rows + row);
        result.cost(matrix_row, col) = map.traversability[id];
        result.height(matrix_row, col) =
            std::isfinite(map.ground_elevation[id])
                ? map.ground_elevation[id]
                : -100.0;
      }
    }
  }
  return result;
}

SnapResult snap(const PlannerCore& planner, const TomogramData& map, int row,
                int col) {
  return planner.snapToTraversable(
      PointXYZ{(row - static_cast<int>(map.rows / 2)) * map.resolution +
                   map.center_x,
               (col - static_cast<int>(map.cols / 2)) * map.resolution +
                   map.center_y,
               0.2f},
      0.2f, 0);
}

void expectRawPathMatchesLegacy(const PlanOutput& output,
                                const Eigen::MatrixXd& legacy_path,
                                const TomogramData& map, float reference_height) {
  ASSERT_EQ(output.path.size(), static_cast<std::size_t>(legacy_path.rows()));
  ASSERT_EQ(output.layers.size(), output.path.size());
  for (int i = 0; i < legacy_path.rows(); ++i) {
    const int layer = static_cast<int>(std::lround(legacy_path(i, 0)));
    const int row = static_cast<int>(std::lround(legacy_path(i, 1)));
    const int col = static_cast<int>(std::lround(legacy_path(i, 2)));
    const PointXYZ& point = output.path[static_cast<std::size_t>(i)];
    EXPECT_EQ(output.layers[static_cast<std::size_t>(i)], layer);
    EXPECT_FLOAT_EQ(
        point.x,
        (row - static_cast<int>(map.rows / 2)) * map.resolution +
            map.center_x);
    EXPECT_FLOAT_EQ(
        point.y,
        (col - static_cast<int>(map.cols / 2)) * map.resolution +
            map.center_y);
    EXPECT_FLOAT_EQ(point.z,
                    map.ground_elevation[map.index(layer, row, col)] +
                        reference_height);
  }
}

TEST(LegacyAStarIntegration, RawPlannerUsesCopiedLegacyAStar) {
  const TomogramData map = testMap();
  PlannerCore planner;
  ASSERT_NO_THROW(planner.load(map, 10.0f, false));

  const SnapResult start = snap(planner, map, 1, 1);
  const SnapResult goal = snap(planner, map, 13, 13);
  ASSERT_TRUE(start.found);
  ASSERT_TRUE(goal.found);

  PlanOutput output;
  ASSERT_TRUE(planner.plan(start, goal, 0.2f, false, output));

  const LegacyMap matrices = legacyMap(map);
  Astar legacy(kDiagonal);
  legacy.Init(49.0, static_cast<int>(map.layers), map.resolution, 0.2,
              matrices.cost, matrices.height, matrices.gateway);
  ASSERT_TRUE(legacy.Search(Eigen::Vector3i(start.layer, 1, 1),
                            Eigen::Vector3i(goal.layer, 13, 13)));
  expectRawPathMatchesLegacy(output, legacy.GetResultMatrix(), map, 0.2f);
}

TEST(LegacyAStarIntegration, DynamicSnapshotUsesCopiedLegacyAStarRules) {
  const TomogramData map = testMap();
  PlannerCore planner;
  ASSERT_NO_THROW(planner.load(map, 10.0f, false));
  const SnapResult start = snap(planner, map, 1, 1);
  const SnapResult goal = snap(planner, map, 13, 13);
  ASSERT_TRUE(start.found);
  ASSERT_TRUE(goal.found);

  std::vector<uint8_t> dynamic(map.cellCount(), 0);
  for (uint32_t col = 0; col < map.cols; ++col) {
    if (col != 7) dynamic[map.index(0, 7, col)] = 100;
  }
  auto snapshot = std::make_shared<DynamicSnapshot>();
  snapshot->cost = dynamic;
  snapshot->lethal_cost = 100;
  planner.setDynamicSnapshot(snapshot);

  PlanOutput output;
  ASSERT_TRUE(planner.plan(start, goal, 0.2f, false, output));

  const LegacyMap matrices = legacyMap(map);
  Astar legacy(kDiagonal);
  legacy.Init(49.0, 1, map.resolution, 0.2, matrices.cost,
              matrices.height, matrices.gateway);
  legacy.SetDynamicCostMap(dynamic.data(), dynamic.size(), 100);
  ASSERT_TRUE(legacy.Search(Eigen::Vector3i(0, 1, 1),
                            Eigen::Vector3i(0, 13, 13)));
  expectRawPathMatchesLegacy(output, legacy.GetResultMatrix(), map, 0.2f);

  std::fill(dynamic.begin() + map.index(0, 7, 0),
            dynamic.begin() + map.index(0, 7, 0) + map.cols, 100);
  snapshot = std::make_shared<DynamicSnapshot>();
  snapshot->cost = dynamic;
  snapshot->lethal_cost = 100;
  planner.setDynamicSnapshot(snapshot);
  legacy.SetDynamicCostMap(dynamic.data(), dynamic.size(), 100);
  EXPECT_FALSE(planner.plan(start, goal, 0.2f, false, output));
  EXPECT_FALSE(legacy.Search(Eigen::Vector3i(0, 1, 1),
                             Eigen::Vector3i(0, 13, 13)));
}

}  // namespace
}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
