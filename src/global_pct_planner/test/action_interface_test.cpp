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

TEST(ActionInterface, GoalCarriesMissionPlanningContext) {
  PlanPath3DGoal goal;
  EXPECT_EQ(0u, goal.mission_id);
  EXPECT_EQ(0u, goal.planning_attempt);
  EXPECT_TRUE(goal.trigger.empty());
  EXPECT_TRUE(goal.trigger_reason.empty());

  goal.mission_id = 42;
  goal.planning_attempt = 3;
  goal.trigger = "local_replan";
  goal.trigger_reason = "local_replan_budget_exhausted";
  EXPECT_EQ(42u, goal.mission_id);
  EXPECT_EQ(3u, goal.planning_attempt);
  EXPECT_EQ("local_replan", goal.trigger);
  EXPECT_EQ("local_replan_budget_exhausted", goal.trigger_reason);
}

TEST(ActionInterface, FeedbackCarriesPlannerTraceContext) {
  PlanPath3DFeedback feedback;
  feedback.request_id = 17;
  feedback.mission_id = 42;
  feedback.planning_attempt = 3;
  feedback.dynamic_retry_count = 1;
  feedback.dynamic_snapshot_version = 9;
  feedback.stage = "planning";
  EXPECT_EQ(17u, feedback.request_id);
  EXPECT_EQ(42u, feedback.mission_id);
  EXPECT_EQ(3u, feedback.planning_attempt);
  EXPECT_EQ(1u, feedback.dynamic_retry_count);
  EXPECT_EQ(9u, feedback.dynamic_snapshot_version);
  EXPECT_EQ("planning", feedback.stage);
}

}  // namespace global_pct_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
