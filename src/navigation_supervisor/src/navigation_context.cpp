#include "navigation_supervisor/navigation_context.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <motion_planner_log/logging.h>

namespace navigation_supervisor {

RecoveryAction recoveryActionFor(FailureDomain domain, FailureCode code) {
  if ((domain == FailureDomain::TF &&
       (code == FailureCode::GOAL_TRANSFORM_FAILED ||
        code == FailureCode::ODOM_TF_UNAVAILABLE)) ||
      (domain == FailureDomain::PCT_PLANNER &&
       code == FailureCode::PCT_ACTION_UNAVAILABLE) ||
      (domain == FailureDomain::SCAN_CONTROLLER &&
       code == FailureCode::SCAN_ACTION_UNAVAILABLE)) {
    return RecoveryAction::WAIT_FOR_ENVIRONMENT;
  }

  if (domain == FailureDomain::PCT_PLANNER &&
      (code == FailureCode::PCT_DYNAMIC_BLOCKED ||
       code == FailureCode::PCT_DYNAMIC_SNAPSHOT_UNSTABLE)) {
    return RecoveryAction::RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE;
  }

  if (domain == FailureDomain::SCAN_CONTROLLER &&
      (code == FailureCode::SCAN_LOCAL_REPLAN_EXHAUSTED ||
       code == FailureCode::SCAN_PROGRESS_STALLED ||
       code == FailureCode::SCAN_GOAL_NOT_REACHED ||
       code == FailureCode::SCAN_ENDPOINT_TRUNCATED)) {
    return RecoveryAction::RETRY_GLOBAL;
  }

  if (domain == FailureDomain::EMERGENCY &&
      code == FailureCode::SCAN_EMERGENCY_STOPPED) {
    return RecoveryAction::RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE;
  }
  return RecoveryAction::TERMINAL;
}

NavigationContext::NavigationContext(ros::NodeHandle nh,
                                     ros::NodeHandle private_nh)
    : nh_(std::move(nh)),
      private_nh_(std::move(private_nh)),
      tf_listener_(tf_buffer_),
      pct_client_(nh_, "/pct/plan_path", true),
      scan_client_(nh_, "/scan/follow_reference_path", true) {
  private_nh_.param<std::string>("navigation_frame", navigation_frame_, "map");
  private_nh_.param("goal_reached_distance", goal_reached_distance_, 0.5);
  private_nh_.param("action_wait_timeout", action_wait_timeout_, 15.0);
  private_nh_.param("action_execution_timeout", action_execution_timeout_, 300.0);
  private_nh_.param("follow_feedback_timeout_sec",
                    follow_feedback_timeout_sec_, 3.0);
  private_nh_.param("mission_deadline_sec", mission_deadline_sec_, 900.0);
  private_nh_.param("emergency_recovery_wait_sec", emergency_recovery_wait_sec_,
                    2.0);
  private_nh_.param("environment_wait_timeout_sec", environment_wait_timeout_sec_,
                    30.0);
  private_nh_.param("max_environment_wait_retries", max_environment_wait_retries_,
                    3);
  private_nh_.param("emergency_recovery_max_attempts",
                    emergency_recovery_max_attempts_, 2);
  private_nh_.param("dynamic_update_timeout_sec", dynamic_update_timeout_sec_,
                    5.0);
  private_nh_.param("require_dynamic_update_after_recovery",
                    require_dynamic_update_after_recovery_, true);
  private_nh_.param("global_replan_max_cycles", global_replan_max_cycles_, 3);
  double max_start_distance = 1.0;
  private_nh_.param("max_start_distance", max_start_distance, 1.0);
  processor_.reset(new ReferencePathProcessor(navigation_frame_,
                                               max_start_distance));

  private_nh_.param<std::string>("status_topic", status_topic_,
                                 "/navigation/status");
  private_nh_.param<std::string>("reference_path_topic", reference_path_topic_,
                                 "/navigation/reference_path");
  private_nh_.param<std::string>("validated_goal_topic", validated_goal_topic_,
                                 "/navigation/validated_goal");
  private_nh_.param<std::string>("body_pose_topic", body_pose_topic_,
                                 "/quad_0/body_pose");
  state_pub_ = nh_.advertise<navigation_msgs::NavigationState>(
      status_topic_, 10, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>(reference_path_topic_, 1, true);
  validated_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      validated_goal_topic_, 1, true);
  odom_sub_ = nh_.subscribe(body_pose_topic_, 10,
                            &NavigationContext::odomCallback, this);
  std::string dynamic_layer_ok_topic = "/pct/dynamic_layer_ok";
  std::string dynamic_layer_enabled_topic = "/pct/dynamic_layer_enabled";
  std::string dynamic_version_topic = "/pct/dynamic_version";
  private_nh_.param<std::string>("dynamic_layer_ok_topic",
                                 dynamic_layer_ok_topic,
                                 dynamic_layer_ok_topic);
  private_nh_.param<std::string>("dynamic_layer_enabled_topic",
                                 dynamic_layer_enabled_topic,
                                 dynamic_layer_enabled_topic);
  private_nh_.param<std::string>("dynamic_version_topic",
                                 dynamic_version_topic,
                                 dynamic_version_topic);
  dynamic_layer_ok_sub_ = nh_.subscribe(
      dynamic_layer_ok_topic, 1,
      &NavigationContext::dynamicLayerOkCallback, this);
  dynamic_layer_enabled_sub_ = nh_.subscribe(
      dynamic_layer_enabled_topic, 1,
      &NavigationContext::dynamicLayerEnabledCallback, this);
  dynamic_version_sub_ = nh_.subscribe(
      dynamic_version_topic, 1,
      &NavigationContext::dynamicVersionCallback, this);

  MOTION_PLANNER_LOG_INFO(
      "Context ready: frame=%s body_pose_topic=%s status_topic=%s "
      "reference_path_topic=%s validated_goal_topic=%s "
      "pct_action=/pct/plan_path scan_action=/scan/follow_reference_path "
      "goal_reached_distance=%.3f max_start_distance=%.3f",
      navigation_frame_.c_str(), body_pose_topic_.c_str(), status_topic_.c_str(),
      reference_path_topic_.c_str(), validated_goal_topic_.c_str(),
      goal_reached_distance_, max_start_distance);
}

void NavigationContext::odomCallback(
    const nav_msgs::OdometryConstPtr& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_ = *message;
  have_odom_ = true;
}

void NavigationContext::dynamicLayerOkCallback(
    const std_msgs::BoolConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  dynamic_layer_state_known_ = true;
  dynamic_layer_healthy_ = message->data;
}

void NavigationContext::dynamicVersionCallback(
    const std_msgs::UInt64ConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  dynamic_layer_version_ = message->data;
}

void NavigationContext::dynamicLayerEnabledCallback(
    const std_msgs::BoolConstPtr& message) {
  if (!message) return;
  std::lock_guard<std::mutex> lock(mutex_);
  dynamic_layer_enabled_known_ = true;
  dynamic_layer_enabled_ = message->data;
}

bool NavigationContext::currentPose(geometry_msgs::PoseStamped& pose) const {
  nav_msgs::Odometry odom;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_odom_) return false;
    odom = odom_;
  }

  geometry_msgs::PoseStamped input;
  input.header = odom.header;
  input.pose = odom.pose.pose;
  return transformPose(input, pose);
}

bool NavigationContext::transformPose(const geometry_msgs::PoseStamped& input,
                                      geometry_msgs::PoseStamped& output) const {
  if (input.header.frame_id.empty()) return false;
  try {
    if (input.header.frame_id == navigation_frame_) {
      output = input;
      return true;
    }
    output = tf_buffer_.transform(input, navigation_frame_, ros::Duration(0.5));
    return true;
  } catch (const tf2::TransformException&) {
    return false;
  }
}

void NavigationContext::beginMission(const geometry_msgs::PoseStamped& goal) {
  raw_goal_ = goal;
  raw_goal_validated_ = false;
  goal_ = geometry_msgs::PoseStamped();
  request_start_ = geometry_msgs::PoseStamped();
  terminal_goal_ = geometry_msgs::PoseStamped();
  execution_goal_ = geometry_msgs::PoseStamped();
  final_pose_ = geometry_msgs::PoseStamped();
  execution_remaining_distance_ = 0.0;
  endpoint_truncated_ = false;
  raw_path_ = nav_msgs::Path();
  processed_path_ = nav_msgs::Path();
  last_error_.clear();
  failure_domain_ = FailureDomain::NONE;
  failure_code_ = FailureCode::NONE;
  global_plan_attempts_ = 0;
  emergency_recovery_attempts_ = 0;
  environment_wait_retries_ = 0;
  pct_planning_snapshot_version_ = 0;
  planning_request_id_ = 0;
  ++mission_id_;
  route_id_ = 0;
  replan_count_ = 0;
  pending_plan_trigger_ = "planner_recovery";
  pending_plan_reason_ = "planner_recovery";
  mission_started_ = ros::WallTime::now();
  last_logged_state_.clear();
  last_logged_reason_.clear();
}

void NavigationContext::setFailure(FailureDomain domain, FailureCode code,
                                    const std::string& message) {
  failure_domain_ = domain;
  failure_code_ = code;
  last_error_ = message;
}

void NavigationContext::clearFailure() {
  failure_domain_ = FailureDomain::NONE;
  failure_code_ = FailureCode::NONE;
  last_error_.clear();
}

void NavigationContext::prepareNextPlanningContext(
    const std::string& trigger, const std::string& reason) {
  pending_plan_trigger_ = trigger.empty() ? "planner_recovery" : trigger;
  pending_plan_reason_ = reason.empty() ? "planner_recovery" : reason;
}

void NavigationContext::recordGlobalPlanRequest() {
  ++global_plan_attempts_;
  if (global_plan_attempts_ > 1) ++replan_count_;
}

bool NavigationContext::globalPlanBudgetAvailable() const {
  return global_plan_attempts_ <
         static_cast<unsigned int>(std::max(1, global_replan_max_cycles_));
}

bool NavigationContext::reserveEmergencyRecovery() {
  if (emergency_recovery_attempts_ >= static_cast<unsigned int>(
          std::max(0, emergency_recovery_max_attempts_))) {
    return false;
  }
  ++emergency_recovery_attempts_;
  return true;
}

bool NavigationContext::reserveEnvironmentWait() {
  if (environment_wait_retries_ >= static_cast<unsigned int>(
          std::max(0, max_environment_wait_retries_))) {
    return false;
  }
  ++environment_wait_retries_;
  return true;
}

bool NavigationContext::missionDeadlineExceeded() const {
  return !mission_started_.isZero() && mission_deadline_sec_ > 0.0 &&
         (ros::WallTime::now() - mission_started_).toSec() >=
             mission_deadline_sec_;
}

bool NavigationContext::dynamicLayerStateKnown() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_layer_state_known_;
}

bool NavigationContext::dynamicLayerHealthy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_layer_healthy_;
}

bool NavigationContext::dynamicLayerEnabledKnown() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_layer_enabled_known_;
}

bool NavigationContext::dynamicLayerEnabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_layer_enabled_;
}

uint64_t NavigationContext::dynamicLayerVersion() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return dynamic_layer_version_;
}

void NavigationContext::setPctPlanningSnapshotVersion(uint64_t version) {
  std::lock_guard<std::mutex> lock(mutex_);
  pct_planning_snapshot_version_ = version;
}

uint64_t NavigationContext::pctPlanningSnapshotVersion() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return pct_planning_snapshot_version_;
}

void NavigationContext::setExecutionResult(
    const geometry_msgs::PoseStamped& execution_goal,
    const geometry_msgs::PoseStamped& final_pose, double remaining_distance,
    bool endpoint_truncated) {
  execution_goal_ = execution_goal;
  final_pose_ = final_pose;
  execution_remaining_distance_ = remaining_distance;
  endpoint_truncated_ = endpoint_truncated;
  if (!execution_goal.header.frame_id.empty()) terminal_goal_ = execution_goal;
}

bool NavigationContext::validateRawGoal(std::string& error) const {
  if (raw_goal_.header.frame_id.empty()) {
    error = "goal_frame_empty";
    return false;
  }
  const auto& p = raw_goal_.pose.position;
  const auto& q = raw_goal_.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w)) {
    error = "goal_non_finite";
    return false;
  }
  return true;
}

bool NavigationContext::setTransformedGoal(std::string& error) {
  // Transform once per mission. Recovery may re-enter this node while waiting
  // for TF, but a successfully transformed goal is immutable for that mission.
  if (!goal_.header.frame_id.empty()) return true;
  if (!transformPose(raw_goal_, goal_)) {
    error = "goal_transform_failed";
    return false;
  }
  publishValidatedGoal();
  return true;
}

bool NavigationContext::processReferencePath(std::string& error) {
  if (!processor_->process(raw_path_, request_start_, processed_path_, error)) {
    return false;
  }
  publishProcessedPath();
  return true;
}

bool NavigationContext::goalReached() const {
  return distanceToGoal() <= goal_reached_distance_;
}

double NavigationContext::distanceToGoal() const {
  geometry_msgs::PoseStamped current;
  if (!currentPose(current) || terminal_goal_.header.frame_id.empty())
    return std::numeric_limits<double>::infinity();
  const auto& a = current.pose.position;
  const auto& b = terminal_goal_.pose.position;
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void NavigationContext::publishState(const std::string& state,
                                     const std::string& reason) {
  navigation_msgs::NavigationState message;
  message.mission_id = mission_id_;
  message.route_id = route_id_;
  message.state = state;
  message.reason = reason;
  state_pub_.publish(message);

  if (state == last_logged_state_ && reason == last_logged_reason_) return;
  last_logged_state_ = state;
  last_logged_reason_ = reason;

  const bool error_state =
      state == "FAILED" || state == "PLANNING_FAILED" ||
      state == "EXECUTION_FAILED" || state == "EMERGENCY_STOPPED" ||
      state == "PATH_REJECTED";
  const bool warning_state =
      state == "WAITING_FOR_POSE" || state == "TIMEOUT" ||
      state == "CANCELED" || state == "PLANNER_RECOVERY" ||
      state == "GLOBAL_REPLANNING";
  const unsigned long long mission_id =
      static_cast<unsigned long long>(mission_id_);
  const unsigned long long route_id = static_cast<unsigned long long>(route_id_);
  const char* reason_text = reason.empty() ? "-" : reason.c_str();
  if (error_state) {
    MOTION_PLANNER_LOG_ERROR(
        "State: mission_id=%llu route_id=%llu state=%s reason=%s", mission_id,
        route_id, state.c_str(), reason_text);
  } else if (warning_state) {
    MOTION_PLANNER_LOG_WARN(
        "State: mission_id=%llu route_id=%llu state=%s reason=%s", mission_id,
        route_id, state.c_str(), reason_text);
  } else {
    MOTION_PLANNER_LOG_INFO(
        "State: mission_id=%llu route_id=%llu state=%s reason=%s", mission_id,
        route_id, state.c_str(), reason_text);
  }
}

void NavigationContext::publishProcessedPath() {
  path_pub_.publish(processed_path_);
}

void NavigationContext::publishValidatedGoal() {
  validated_goal_pub_.publish(goal_);
}

void NavigationContext::cancelActiveActions() {
  pct_client_.cancelGoal();
  scan_client_.cancelGoal();
}

}  // namespace navigation_supervisor
