#include <gtest/gtest.h>

#include "navigation_supervisor/navigation_context.h"

namespace navigation_supervisor {

TEST(RecoveryPolicy, WaitsForTransientPoseAndActionAvailability) {
  EXPECT_EQ(RecoveryAction::WAIT_FOR_ENVIRONMENT,
            recoveryActionFor(FailureDomain::TF,
                              FailureCode::ODOM_TF_UNAVAILABLE));
  EXPECT_EQ(RecoveryAction::WAIT_FOR_ENVIRONMENT,
            recoveryActionFor(FailureDomain::PCT_PLANNER,
                              FailureCode::PCT_ACTION_UNAVAILABLE));
  EXPECT_EQ(RecoveryAction::WAIT_FOR_ENVIRONMENT,
            recoveryActionFor(FailureDomain::SCAN_CONTROLLER,
                              FailureCode::SCAN_ACTION_UNAVAILABLE));
}

TEST(RecoveryPolicy, RetriesDynamicFailuresAfterPerceptionRefresh) {
  EXPECT_EQ(RecoveryAction::RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE,
            recoveryActionFor(FailureDomain::PCT_PLANNER,
                              FailureCode::PCT_DYNAMIC_BLOCKED));
  EXPECT_EQ(RecoveryAction::RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE,
            recoveryActionFor(FailureDomain::PCT_PLANNER,
                              FailureCode::PCT_DYNAMIC_SNAPSHOT_UNSTABLE));
  EXPECT_EQ(RecoveryAction::RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE,
            recoveryActionFor(FailureDomain::EMERGENCY,
                              FailureCode::SCAN_EMERGENCY_STOPPED));
}

TEST(RecoveryPolicy, EscalatesExhaustedLocalRecoveryToGlobalPlanning) {
  EXPECT_EQ(RecoveryAction::RETRY_GLOBAL,
            recoveryActionFor(FailureDomain::SCAN_CONTROLLER,
                              FailureCode::SCAN_LOCAL_REPLAN_EXHAUSTED));
  EXPECT_EQ(RecoveryAction::RETRY_GLOBAL,
            recoveryActionFor(FailureDomain::SCAN_CONTROLLER,
                              FailureCode::SCAN_PROGRESS_STALLED));
  EXPECT_EQ(RecoveryAction::RETRY_GLOBAL,
            recoveryActionFor(FailureDomain::SCAN_CONTROLLER,
                              FailureCode::SCAN_GOAL_NOT_REACHED));
}

TEST(RecoveryPolicy, DoesNotRetryStaticOrMalformedPlanningFailures) {
  EXPECT_EQ(RecoveryAction::TERMINAL,
            recoveryActionFor(FailureDomain::PCT_PLANNER,
                              FailureCode::PCT_NO_PATH));
  EXPECT_EQ(RecoveryAction::TERMINAL,
            recoveryActionFor(FailureDomain::PCT_PLANNER,
                              FailureCode::PCT_OUT_OF_MAP));
  EXPECT_EQ(RecoveryAction::TERMINAL,
            recoveryActionFor(FailureDomain::PCT_PLANNER,
                              FailureCode::PCT_NO_TRAVERSABLE_LAYER));
  EXPECT_EQ(RecoveryAction::TERMINAL,
            recoveryActionFor(FailureDomain::GOAL,
                              FailureCode::INVALID_GOAL));
}

}  // namespace navigation_supervisor
