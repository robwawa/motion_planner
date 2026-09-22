#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "global_pct_planner/dynamic_obstacle_layer.h"
#include "global_pct_planner/planner_core.h"

namespace global_pct_planner {
namespace {

TomogramData plannerMap() {
  TomogramData map;
  map.resolution = 1.0f;
  map.center_x = 10.0f;
  map.center_y = -4.0f;
  map.slice_h0 = 0.0f;
  map.slice_dh = 1.0f;
  map.layers = 1;
  map.rows = 9;
  map.cols = 9;
  const std::size_t count = map.cellCount();
  map.traversability.assign(count, 0.0f);
  map.traversability_grad_x.assign(count, 0.0f);
  map.traversability_grad_y.assign(count, 0.0f);
  map.ground_elevation.assign(count, 0.0f);
  map.ceiling_elevation.assign(count, 2.0f);
  return map;
}

}  // namespace

TEST(PlannerCore, SnapsAndPlansWithLegacyAStar) {
  PlannerCore planner;
  const TomogramData map = plannerMap();
  ASSERT_NO_THROW(planner.load(map, 10.0f, false));

  const SnapResult start = planner.snapToTraversable(
      PointXYZ{map.center_x - 3.0f, map.center_y - 3.0f, 0.2f}, 0.2f, 1);
  const SnapResult goal = planner.snapToTraversable(
      PointXYZ{map.center_x + 3.0f, map.center_y + 3.0f, 0.2f}, 0.2f, 1);
  ASSERT_TRUE(start.found);
  ASSERT_TRUE(goal.found);

  PlanOutput output;
  ASSERT_TRUE(planner.plan(start, goal, 0.2f, false, output));
  ASSERT_FALSE(output.path.empty());
  EXPECT_FLOAT_EQ(output.path.back().x, goal.x);
  EXPECT_FLOAT_EQ(output.path.back().y, goal.y);
  EXPECT_EQ(output.path.size(), output.layers.size());
  for (const PointXYZ& point : output.path) {
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
    EXPECT_TRUE(std::isfinite(point.z));
  }
}

TEST(PlannerCore, DynamicSnapshotCanBlockTheOnlyLayer) {
  PlannerCore planner;
  const TomogramData map = plannerMap();
  ASSERT_NO_THROW(planner.load(map, 10.0f, false));
  const SnapResult start = planner.snapToTraversable(
      PointXYZ{map.center_x - 3.0f, map.center_y - 3.0f, 0.2f}, 0.2f, 1);
  const SnapResult goal = planner.snapToTraversable(
      PointXYZ{map.center_x + 3.0f, map.center_y + 3.0f, 0.2f}, 0.2f, 1);
  ASSERT_TRUE(start.found);
  ASSERT_TRUE(goal.found);

  auto blocked = std::make_shared<DynamicSnapshot>();
  blocked->cost.assign(map.cellCount(), 100);
  blocked->lethal_cost = 100;
  planner.setDynamicSnapshot(blocked);
  PlanOutput output;
  EXPECT_FALSE(planner.plan(start, goal, 0.2f, false, output));
}

TEST(PlannerCore, GatewayMatchesLegacyNanToNumAtNanBoundary) {
  TomogramData map;
  map.resolution = 1.0f;
  map.center_x = 0.0f;
  map.center_y = 0.0f;
  map.slice_h0 = 0.0f;
  map.slice_dh = 1.0f;
  map.layers = 2;
  map.rows = 3;
  map.cols = 3;
  const std::size_t count = map.cellCount();
  map.traversability.assign(count, 0.0f);
  map.traversability_grad_x.assign(count, 0.0f);
  map.traversability_grad_y.assign(count, 0.0f);
  map.ground_elevation.assign(count, 0.0f);
  map.ceiling_elevation.assign(count, 2.0f);

  // planner_wrapper.py first applies np.nan_to_num(elev_g_raw, nan=-100),
  // then computes the downward gateway from the traversability difference.
  // Both sides being NaN therefore have equal converted ground heights and
  // must still produce the same gateway as the legacy implementation.
  const std::size_t lower = map.index(0, 1, 1);
  const std::size_t upper = map.index(1, 1, 1);
  map.ground_elevation[lower] = std::numeric_limits<float>::quiet_NaN();
  map.ground_elevation[upper] = std::numeric_limits<float>::quiet_NaN();
  map.traversability[upper] = 20.0f;

  PlannerCore planner;
  ASSERT_NO_THROW(planner.load(map, 10.0f, false));

  // Start at the upper gateway cell.  A downward gateway (-2) makes the
  // copied legacy A* expand the next planar neighbor on layer 0.
  const SnapResult start{true, 0.0f, 0.0f, -99.8f, 1, 0.0f};
  const SnapResult goal{true, 0.0f, 1.0f, 0.2f, 0, 0.0f};
  PlanOutput output;
  EXPECT_TRUE(planner.plan(start, goal, 0.2f, false, output));
  ASSERT_FALSE(output.path.empty());
  EXPECT_EQ(output.layers.back(), 0);
}

TEST(PlannerCore, OptimizedPathUsesLegacyWorldCoordinateConvention) {
  PlannerCore planner;
  const TomogramData map = plannerMap();
  ASSERT_NO_THROW(planner.load(map, 10.0f, false));

  const SnapResult start = planner.snapToTraversable(
      PointXYZ{map.center_x - 3.0f, map.center_y - 3.0f, 0.2f}, 0.2f, 1);
  const SnapResult goal = planner.snapToTraversable(
      PointXYZ{map.center_x + 3.0f, map.center_y + 3.0f, 0.2f}, 0.2f, 1);
  ASSERT_TRUE(start.found);
  ASSERT_TRUE(goal.found);

  PlanOutput output;
  ASSERT_TRUE(planner.plan(start, goal, 0.2f, true, output));
  ASSERT_GE(output.path.size(), 2u);

  // The native optimizer stores x=column and y=row.  The legacy wrapper
  // exports world x from row and world y from column, with floor(map_size/2)
  // offsets.  GPMP may move a prior endpoint slightly during optimization,
  // so check the exported coordinate frame and path direction rather than
  // requiring exact endpoint equality.
  const float min_x = map.center_x - static_cast<float>(map.rows / 2) - 1.0f;
  const float max_x = map.center_x +
                     static_cast<float>(map.rows - 1 - map.rows / 2) + 1.0f;
  const float min_y = map.center_y - static_cast<float>(map.cols / 2) - 1.0f;
  const float max_y = map.center_y +
                     static_cast<float>(map.cols - 1 - map.cols / 2) + 1.0f;
  for (const PointXYZ& point : output.path) {
    EXPECT_GE(point.x, min_x);
    EXPECT_LE(point.x, max_x);
    EXPECT_GE(point.y, min_y);
    EXPECT_LE(point.y, max_y);
  }
  EXPECT_GT(output.path.back().x, output.path.front().x);
  EXPECT_GT(output.path.back().y, output.path.front().y);
}

TEST(PlannerCore, QuinticOptimizedPathUsesLegacyWorldCoordinateConvention) {
  PlannerCore planner;
  const TomogramData map = plannerMap();
  ASSERT_NO_THROW(planner.load(map, 10.0f, true));

  const SnapResult start = planner.snapToTraversable(
      PointXYZ{map.center_x - 3.0f, map.center_y - 3.0f, 0.2f}, 0.2f, 1);
  const SnapResult goal = planner.snapToTraversable(
      PointXYZ{map.center_x + 3.0f, map.center_y + 3.0f, 0.2f}, 0.2f, 1);
  ASSERT_TRUE(start.found);
  ASSERT_TRUE(goal.found);

  PlanOutput output;
  ASSERT_TRUE(planner.plan(start, goal, 0.2f, true, output));
  ASSERT_GE(output.path.size(), 2u);
  EXPECT_GT(output.path.back().x, output.path.front().x);
  EXPECT_GT(output.path.back().y, output.path.front().y);
  for (const PointXYZ& point : output.path) {
    EXPECT_GE(point.x, map.center_x - static_cast<float>(map.rows / 2) - 1.0f);
    EXPECT_LE(point.x, map.center_x +
                          static_cast<float>(map.rows - 1 - map.rows / 2) +
                          1.0f);
    EXPECT_GE(point.y, map.center_y - static_cast<float>(map.cols / 2) - 1.0f);
    EXPECT_LE(point.y, map.center_y +
                          static_cast<float>(map.cols - 1 - map.cols / 2) +
                          1.0f);
  }
}

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
