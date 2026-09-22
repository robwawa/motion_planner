#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "global_pct_planner/dynamic_obstacle_layer.h"

namespace global_pct_planner {
namespace {

TomogramData testMap() {
  TomogramData map;
  map.resolution = 0.1f;
  map.center_x = 0.0f;
  map.center_y = 0.0f;
  map.layers = 2;
  map.rows = 41;
  map.cols = 41;
  const std::size_t count = map.cellCount();
  map.traversability.assign(count, 0.0f);
  map.traversability_grad_x.assign(count, 0.0f);
  map.traversability_grad_y.assign(count, 0.0f);
  map.ground_elevation.resize(count);
  map.ceiling_elevation.resize(count);
  for (uint32_t layer = 0; layer < map.layers; ++layer) {
    for (uint32_t row = 0; row < map.rows; ++row) {
      for (uint32_t col = 0; col < map.cols; ++col) {
        const std::size_t index = map.index(layer, row, col);
        map.ground_elevation[index] = layer == 0 ? 0.0f : 3.0f;
        map.ceiling_elevation[index] = layer == 0 ? 1.0f : 4.0f;
      }
    }
  }
  return map;
}

}  // namespace

TEST(DynamicObstacleLayer, ReplacesSnapshotsAndKeepsOldSnapshotImmutable) {
  const TomogramData map = testMap();
  DynamicObstacleLayer layer(map, 49.0f, 100, 0.08f, 0.6f, 0.25f, 0.32f,
                             0.15f, 0.5f, 100, 128.0);
  const auto initial = layer.snapshot();
  EXPECT_EQ(initial->version, 0u);
  EXPECT_TRUE(layer.replaceSnapshot({{0.0f, 0.0f, 0.3f}}));
  const auto first = layer.snapshot();
  ASSERT_EQ(first->version, 1u);
  EXPECT_GT(first->cost[map.index(0, 20, 20)], 0u);
  EXPECT_EQ(initial->cost[map.index(0, 20, 20)], 0u);
  EXPECT_FALSE(layer.replaceSnapshot({{0.0f, 0.0f, 0.3f}}));
  EXPECT_EQ(layer.snapshot()->version, 1u);

  EXPECT_TRUE(layer.replaceSnapshot({{1.5f, 0.0f, 0.3f}}));
  const auto second = layer.snapshot();
  EXPECT_EQ(second->version, 2u);
  EXPECT_EQ(first->cost[map.index(0, 20, 20)], 100u);
  EXPECT_EQ(second->cost[map.index(0, 20, 20)], 0u);
  EXPECT_FALSE(layer.assignedPoints().empty());
  EXPECT_TRUE(layer.clear());
  EXPECT_EQ(layer.snapshot()->version, 3u);
  EXPECT_TRUE(std::all_of(layer.snapshot()->cost.begin(),
                          layer.snapshot()->cost.end(),
                          [](uint8_t value) { return value == 0; }));
}

TEST(DynamicObstacleLayer, RejectsInvalidMemoryAndIgnoresWrongHeights) {
  const TomogramData map = testMap();
  EXPECT_THROW(
      DynamicObstacleLayer(map, 49.0f, 100, 0.08f, 0.6f, 0.25f, 0.32f, 0.15f,
                           0.5f, 100, 0.0001),
      std::length_error);
  DynamicObstacleLayer layer(map, 49.0f, 100, 0.08f, 0.6f, 0.25f, 0.32f,
                             0.15f, 0.5f, 100, 128.0);
  EXPECT_FALSE(layer.replaceSnapshot({{0.0f, 0.0f, 2.0f}}));
  EXPECT_EQ(layer.snapshot()->version, 0u);
}

TEST(DynamicObstacleLayer, UsesLegacyNanToNumElevationForProjection) {
  TomogramData map = testMap();
  const std::size_t center = map.index(0, map.rows / 2, map.cols / 2);
  map.ground_elevation[center] = std::numeric_limits<float>::quiet_NaN();
  map.ceiling_elevation[center] = std::numeric_limits<float>::quiet_NaN();

  DynamicObstacleLayer layer(map, 49.0f, 100, 0.08f, 0.6f, 0.25f, 0.32f,
                             0.15f, 0.5f, 100, 128.0);
  // Legacy Python converts NaN ground/ceiling to -100/1e6 before dynamic
  // projection.  This point is therefore valid for the converted layer.
  EXPECT_TRUE(layer.replaceSnapshot({{0.0f, 0.0f, -99.8f}}));
  EXPECT_EQ(layer.assignedPoints().size(), 1u);
  EXPECT_EQ(layer.assignedPoints().front().layer, 0);
  EXPECT_EQ(layer.assignedPoints().front().row,
            static_cast<int>(map.rows / 2));
  EXPECT_EQ(layer.assignedPoints().front().col,
            static_cast<int>(map.cols / 2));
}

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
