#include <gtest/gtest.h>

#include <global_pct_planner/PlanPath3DAction.h>

namespace global_pct_planner {

TEST(ActionInterface, StatusValuesRemainStable) {
  EXPECT_EQ(0u, PlanPath3DResult::SUCCESS);
  EXPECT_EQ(1u, PlanPath3DResult::INVALID_REQUEST);
  EXPECT_EQ(2u, PlanPath3DResult::FRAME_MISMATCH);
  EXPECT_EQ(3u, PlanPath3DResult::OUT_OF_MAP);
  EXPECT_EQ(4u, PlanPath3DResult::NO_TRAVERSABLE_LAYER);
  EXPECT_EQ(5u, PlanPath3DResult::NO_PATH);
  EXPECT_EQ(6u, PlanPath3DResult::PREEMPTED);
  EXPECT_EQ(7u, PlanPath3DResult::INTERNAL_ERROR);
}

TEST(ActionInterface, ResultCarriesSnappedEndpointState) {
  PlanPath3DResult result;
  result.status = PlanPath3DResult::SUCCESS;
  result.has_snapped_start = true;
  result.has_snapped_goal = true;
  result.snapped_start_distance = 0.25;
  result.snapped_goal_distance = 0.5;
  EXPECT_TRUE(result.has_snapped_start);
  EXPECT_TRUE(result.has_snapped_goal);
  EXPECT_DOUBLE_EQ(0.25, result.snapped_start_distance);
  EXPECT_DOUBLE_EQ(0.5, result.snapped_goal_distance);
}

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
