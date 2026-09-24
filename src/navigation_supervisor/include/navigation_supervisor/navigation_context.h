#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <global_pct_planner/PlanPath3DAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <navigation_msgs/FollowReferencePathAction.h>
#include <navigation_msgs/NavigationState.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "navigation_supervisor/reference_path_processor.h"

namespace navigation_supervisor {

enum class FailureDomain : uint8_t {
  NONE = 0,
  GOAL,
  TF,
  PCT_PLANNER,
  REFERENCE_PATH,
  SCAN_CONTROLLER,
  TIMEOUT,
  CANCELED,
  EMERGENCY
};

enum class FailureCode : uint16_t {
  NONE = 0,
  INVALID_GOAL,
  GOAL_TRANSFORM_FAILED,
  ODOM_TF_UNAVAILABLE,
  PCT_ACTION_UNAVAILABLE,
  PCT_INVALID_REQUEST,
  PCT_FRAME_MISMATCH,
  PCT_OUT_OF_MAP,
  PCT_NO_TRAVERSABLE_LAYER,
  PCT_NO_PATH,
  PCT_DYNAMIC_BLOCKED,
  PCT_DYNAMIC_SNAPSHOT_UNSTABLE,
  PCT_INTERNAL_ERROR,
  PCT_PREEMPTED,
  PCT_NO_RESULT,
  RAW_PATH_TOO_SHORT,
  GLOBAL_REPLAN_EXHAUSTED,
  PATH_FRAME_MISMATCH,
  PATH_TOO_SHORT,
  START_TOO_FAR,
  PATH_NON_FINITE,
  PATH_ALREADY_COMPLETE,
  PROCESSED_PATH_INVALID,
  SCAN_ACTION_UNAVAILABLE,
  SCAN_LOCAL_REPLAN_EXHAUSTED,
  SCAN_PROGRESS_STALLED,
  SCAN_GOAL_NOT_REACHED,
  SCAN_ENDPOINT_TRUNCATED,
  SCAN_REJECTED,
  SCAN_FAILED,
  SCAN_CANCELED,
  SCAN_EMERGENCY_STOPPED,
  EMERGENCY_RECOVERY_EXHAUSTED,
  ACTION_TIMEOUT,
  MISSION_DEADLINE,
  ENVIRONMENT_WAIT_EXHAUSTED,
  DYNAMIC_PERCEPTION_UNAVAILABLE,
  GENERIC
};

enum class RecoveryAction : uint8_t {
  TERMINAL = 0,
  WAIT_FOR_ENVIRONMENT,
  RETRY_GLOBAL,
  RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE
};

RecoveryAction recoveryActionFor(FailureDomain domain, FailureCode code);

class NavigationContext {
 public:
  using PctClient =
      actionlib::SimpleActionClient<global_pct_planner::PlanPath3DAction>;
  using ScanClient =
      actionlib::SimpleActionClient<navigation_msgs::FollowReferencePathAction>;

  NavigationContext(ros::NodeHandle nh, ros::NodeHandle private_nh);

  void odomCallback(const nav_msgs::OdometryConstPtr& message);
  void dynamicLayerOkCallback(const std_msgs::BoolConstPtr& message);
  void dynamicLayerEnabledCallback(const std_msgs::BoolConstPtr& message);
  void dynamicVersionCallback(const std_msgs::UInt64ConstPtr& message);
  bool currentPose(geometry_msgs::PoseStamped& pose) const;
  bool transformPose(const geometry_msgs::PoseStamped& input,
                     geometry_msgs::PoseStamped& output) const;

  void beginMission(const geometry_msgs::PoseStamped& goal);
  bool setTransformedGoal(std::string& error);
  bool validateRawGoal(std::string& error) const;
  bool rawGoalValidated() const { return raw_goal_validated_; }
  void markRawGoalValidated() { raw_goal_validated_ = true; }

  const geometry_msgs::PoseStamped& rawGoal() const { return raw_goal_; }
  const geometry_msgs::PoseStamped& goal() const { return goal_; }
  const geometry_msgs::PoseStamped& requestStart() const {
    return request_start_;
  }
  const geometry_msgs::PoseStamped& terminalGoal() const {
    return terminal_goal_;
  }
  const nav_msgs::Path& rawPath() const { return raw_path_; }
  const nav_msgs::Path& processedPath() const { return processed_path_; }

  void setRequestStart(const geometry_msgs::PoseStamped& pose) {
    request_start_ = pose;
  }
  void setRawPath(const nav_msgs::Path& path) { raw_path_ = path; }
  void setProcessedPath(const nav_msgs::Path& path) { processed_path_ = path; }
  void setTerminalGoal(const geometry_msgs::PoseStamped& pose) {
    terminal_goal_ = pose;
  }

  bool processReferencePath(std::string& error);
  bool goalReached() const;
  double distanceToGoal() const;

  void publishState(const std::string& state, const std::string& reason = "");
  void publishProcessedPath();
  void publishValidatedGoal();

  void cancelActiveActions();
  void clearCurrentRoute() {
    raw_path_ = nav_msgs::Path();
    processed_path_ = nav_msgs::Path();
  }
  void setExecutionResult(const geometry_msgs::PoseStamped& execution_goal,
                          const geometry_msgs::PoseStamped& final_pose,
                          double remaining_distance, bool endpoint_truncated);
  const geometry_msgs::PoseStamped& executionGoal() const {
    return execution_goal_;
  }
  const geometry_msgs::PoseStamped& finalPose() const { return final_pose_; }
  double executionRemainingDistance() const { return execution_remaining_distance_; }
  bool endpointTruncated() const { return endpoint_truncated_; }
  void setError(const std::string& error) {
    last_error_ = error;
    failure_domain_ = FailureDomain::NONE;
    failure_code_ = FailureCode::GENERIC;
  }
  void setFailure(FailureDomain domain, FailureCode code,
                  const std::string& message);
  void clearFailure();
  const std::string& lastError() const { return last_error_; }
  FailureDomain failureDomain() const { return failure_domain_; }
  FailureCode failureCode() const { return failure_code_; }

  // A navigation cycle is one complete attempt through the behavior tree.
  // The configured maximum includes the first attempt, while replanCount()
  // reports only additional global planning requests.
  void prepareNextPlanningContext(const std::string& trigger,
                                  const std::string& reason);
  void recordGlobalPlanRequest();
  bool globalPlanBudgetAvailable() const;
  bool reserveEmergencyRecovery();
  bool reserveEnvironmentWait();
  bool missionDeadlineExceeded() const;
  bool dynamicLayerStateKnown() const;
  bool dynamicLayerHealthy() const;
  bool dynamicLayerEnabledKnown() const;
  bool dynamicLayerEnabled() const;
  uint64_t dynamicLayerVersion() const;
  void setPctPlanningSnapshotVersion(uint64_t version);
  uint64_t pctPlanningSnapshotVersion() const;

  uint64_t missionId() const { return mission_id_; }
  uint64_t nextRouteId() { return ++route_id_; }
  uint64_t routeId() const { return route_id_; }
  unsigned int replanCount() const { return replan_count_; }
  unsigned int planningAttempt() const { return global_plan_attempts_; }
  void setPlanningRequestId(uint64_t request_id) {
    planning_request_id_ = request_id;
  }
  uint64_t planningRequestId() const { return planning_request_id_; }
  std::string planningTrigger() const {
    return global_plan_attempts_ <= 1 ? "initial"
                                      : pending_plan_trigger_;
  }
  std::string planningReason() const {
    return global_plan_attempts_ <= 1 ? "initial_goal" : pending_plan_reason_;
  }
  double missionDurationMs() const {
    return mission_started_.isZero()
               ? 0.0
               : (ros::WallTime::now() - mission_started_).toSec() * 1000.0;
  }

  double goalReachedDistance() const { return goal_reached_distance_; }
  double actionWaitTimeout() const { return action_wait_timeout_; }
  double actionExecutionTimeout() const { return action_execution_timeout_; }
  double followFeedbackTimeoutSec() const { return follow_feedback_timeout_sec_; }
  double emergencyRecoveryWaitSec() const { return emergency_recovery_wait_sec_; }
  double missionDeadlineSec() const { return mission_deadline_sec_; }
  double environmentWaitTimeoutSec() const { return environment_wait_timeout_sec_; }
  int maxEnvironmentWaitRetries() const { return max_environment_wait_retries_; }
  int maxEmergencyRecoveryAttempts() const { return emergency_recovery_max_attempts_; }
  double dynamicUpdateTimeoutSec() const { return dynamic_update_timeout_sec_; }
  bool requireDynamicUpdateAfterRecovery() const {
    return require_dynamic_update_after_recovery_;
  }
  int maxReplanCycles() const { return global_replan_max_cycles_; }
  const std::string& navigationFrame() const { return navigation_frame_; }

  PctClient& pctClient() { return pct_client_; }
  ScanClient& scanClient() { return scan_client_; }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::string navigation_frame_;
  double goal_reached_distance_{0.5};
  double action_wait_timeout_{15.0};
  double action_execution_timeout_{300.0};
  double follow_feedback_timeout_sec_{3.0};
  double mission_deadline_sec_{900.0};
  double emergency_recovery_wait_sec_{2.0};
  double environment_wait_timeout_sec_{30.0};
  double dynamic_update_timeout_sec_{5.0};
  int max_environment_wait_retries_{3};
  int emergency_recovery_max_attempts_{2};
  int global_replan_max_cycles_{3};
  bool require_dynamic_update_after_recovery_{true};

  mutable std::mutex mutex_;
  nav_msgs::Odometry odom_;
  bool have_odom_{false};

  geometry_msgs::PoseStamped raw_goal_;
  bool raw_goal_validated_{false};
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped request_start_;
  geometry_msgs::PoseStamped terminal_goal_;
  geometry_msgs::PoseStamped execution_goal_;
  geometry_msgs::PoseStamped final_pose_;
  double execution_remaining_distance_{0.0};
  bool endpoint_truncated_{false};
  nav_msgs::Path raw_path_;
  nav_msgs::Path processed_path_;
  std::string last_error_;
  uint64_t mission_id_{0};
  uint64_t route_id_{0};
  unsigned int replan_count_{0};
  unsigned int global_plan_attempts_{0};
  unsigned int emergency_recovery_attempts_{0};
  unsigned int environment_wait_retries_{0};
  bool dynamic_layer_state_known_{false};
  bool dynamic_layer_healthy_{false};
  bool dynamic_layer_enabled_known_{false};
  bool dynamic_layer_enabled_{false};
  uint64_t dynamic_layer_version_{0};
  uint64_t pct_planning_snapshot_version_{0};
  uint64_t planning_request_id_{0};
  std::string pending_plan_trigger_{"planner_recovery"};
  std::string pending_plan_reason_{"planner_recovery"};
  ros::WallTime mission_started_;
  FailureDomain failure_domain_{FailureDomain::NONE};
  FailureCode failure_code_{FailureCode::NONE};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<ReferencePathProcessor> processor_;
  PctClient pct_client_;
  ScanClient scan_client_;

  ros::Publisher state_pub_;
  ros::Publisher path_pub_;
  ros::Publisher validated_goal_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber dynamic_layer_ok_sub_;
  ros::Subscriber dynamic_layer_enabled_sub_;
  ros::Subscriber dynamic_version_sub_;

  std::string body_pose_topic_;
  std::string status_topic_;
  std::string reference_path_topic_;
  std::string validated_goal_topic_;
  std::string last_logged_state_;
  std::string last_logged_reason_;
};

}  // namespace navigation_supervisor
