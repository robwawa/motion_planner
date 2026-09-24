#include "navigation_supervisor/bt_nodes.h"

#include <cmath>
#include <cctype>
#include <string>

#include <boost/bind/bind.hpp>
#include <global_pct_planner/PlanPath3DAction.h>
#include <motion_planner_log/logging.h>
#include <navigation_msgs/FollowReferencePathAction.h>

namespace navigation_supervisor {

namespace {

std::string normalizeLogToken(const std::string& value,
                              const std::string& fallback) {
  if (value.empty()) return fallback;
  std::string output;
  output.reserve(value.size());
  for (const char character : value) {
    const unsigned char byte = static_cast<unsigned char>(character);
    if (std::isalnum(byte) || character == '_' || character == '-') {
      output.push_back(static_cast<char>(std::tolower(byte)));
    } else {
      output.push_back('_');
    }
  }
  return output.empty() ? fallback : output;
}

void setGoalFailure(NavigationContext* context, const std::string& error) {
  context->setFailure(FailureDomain::GOAL, FailureCode::INVALID_GOAL, error);
}

void setTransformFailure(NavigationContext* context, const std::string& error) {
  context->setFailure(FailureDomain::TF, FailureCode::GOAL_TRANSFORM_FAILED,
                      error);
}

void setPoseFailure(NavigationContext* context, const std::string& error) {
  context->setFailure(FailureDomain::TF, FailureCode::ODOM_TF_UNAVAILABLE,
                      error);
}

void setPathFailure(NavigationContext* context, const std::string& error) {
  FailureCode code = FailureCode::GENERIC;
  if (error.find("frame") != std::string::npos)
    code = FailureCode::PATH_FRAME_MISMATCH;
  else if (error.find("too_far") != std::string::npos)
    code = FailureCode::START_TOO_FAR;
  else if (error.find("non_finite") != std::string::npos)
    code = FailureCode::PATH_NON_FINITE;
  else if (error.find("already_complete") != std::string::npos)
    code = FailureCode::PATH_ALREADY_COMPLETE;
  else if (error.find("fewer") != std::string::npos ||
           error.find("two") != std::string::npos)
    code = FailureCode::PATH_TOO_SHORT;
  context->setFailure(FailureDomain::REFERENCE_PATH, code, error);
}

void setPctFailure(NavigationContext* context, uint8_t status,
                   const std::string& reason_code,
                   const std::string& message) {
  using Result = global_pct_planner::PlanPath3DResult;
  FailureDomain domain = FailureDomain::PCT_PLANNER;
  FailureCode code = FailureCode::PCT_INTERNAL_ERROR;
  if (reason_code == "dynamic_obstacle_blocked") {
    code = FailureCode::PCT_DYNAMIC_BLOCKED;
  } else if (reason_code == "dynamic_snapshot_unstable") {
    code = FailureCode::PCT_DYNAMIC_SNAPSHOT_UNSTABLE;
  } else if (reason_code == "no_traversable_surface") {
    code = FailureCode::PCT_NO_TRAVERSABLE_LAYER;
  } else if (reason_code == "pct_no_path_static" ||
             reason_code == "path_too_short") {
    code = FailureCode::PCT_NO_PATH;
  } else {
    switch (status) {
    case Result::INVALID_REQUEST:
      code = FailureCode::PCT_INVALID_REQUEST;
      break;
    case Result::FRAME_MISMATCH:
      code = FailureCode::PCT_FRAME_MISMATCH;
      break;
    case Result::OUT_OF_MAP:
      code = FailureCode::PCT_OUT_OF_MAP;
      break;
    case Result::NO_TRAVERSABLE_LAYER:
      code = FailureCode::PCT_NO_TRAVERSABLE_LAYER;
      break;
    case Result::NO_PATH:
      code = FailureCode::PCT_NO_PATH;
      break;
    case Result::PREEMPTED:
      domain = FailureDomain::CANCELED;
      code = FailureCode::PCT_PREEMPTED;
      break;
    case Result::INTERNAL_ERROR:
      code = FailureCode::PCT_INTERNAL_ERROR;
      break;
    default:
      code = FailureCode::PCT_NO_RESULT;
      break;
    }
  }
  context->setFailure(domain, code,
                      message.empty() ? "pct_planning_failed" : message);
}

std::string recoveryReasonCode(FailureDomain domain, FailureCode code) {
  switch (code) {
    case FailureCode::GOAL_TRANSFORM_FAILED:
      return "goal_transform_unavailable";
    case FailureCode::ODOM_TF_UNAVAILABLE:
      return "robot_pose_unavailable";
    case FailureCode::PCT_ACTION_UNAVAILABLE:
      return "pct_action_unavailable";
    case FailureCode::SCAN_ACTION_UNAVAILABLE:
      return "scan_action_unavailable";
    case FailureCode::PCT_DYNAMIC_BLOCKED:
      return "dynamic_obstacle_blocked";
    case FailureCode::PCT_DYNAMIC_SNAPSHOT_UNSTABLE:
      return "dynamic_snapshot_unstable";
    case FailureCode::SCAN_LOCAL_REPLAN_EXHAUSTED:
      return "local_replan_exhausted";
    case FailureCode::SCAN_PROGRESS_STALLED:
      return "progress_stalled";
    case FailureCode::SCAN_GOAL_NOT_REACHED:
      return "goal_not_reached";
    case FailureCode::SCAN_EMERGENCY_STOPPED:
      return "trajectory_collision";
    default:
      return domain == FailureDomain::EMERGENCY ? "emergency_recovery"
                                                 : "planner_recovery";
  }
}

}  // namespace

ValidateGoalNode::ValidateGoalNode(const std::string& name,
                                   const BT::NodeConfiguration& config,
                                   NavigationContext* context)
    : BT::SyncActionNode(name, config), context_(context) {}

BT::NodeStatus ValidateGoalNode::tick() {
  if (context_->rawGoalValidated()) return BT::NodeStatus::SUCCESS;
  std::string error;
  if (!context_->validateRawGoal(error)) {
    setGoalFailure(context_, error);
    context_->publishState("FAILED", error);
    return BT::NodeStatus::FAILURE;
  }
  context_->markRawGoalValidated();
  context_->clearFailure();
  context_->publishState("VALIDATING");
  return BT::NodeStatus::SUCCESS;
}

TransformGoalNode::TransformGoalNode(const std::string& name,
                                     const BT::NodeConfiguration& config,
                                     NavigationContext* context)
    : BT::SyncActionNode(name, config), context_(context) {}

BT::NodeStatus TransformGoalNode::tick() {
  std::string error;
  if (!context_->setTransformedGoal(error)) {
    setTransformFailure(context_, error);
    context_->publishState("FAILED", error);
    return BT::NodeStatus::FAILURE;
  }
  context_->clearFailure();
  MOTION_PLANNER_LOG_DEBUG(
      "Goal transformed: mission_id=%llu frame=%s",
      static_cast<unsigned long long>(context_->missionId()),
      context_->goal().header.frame_id.c_str());
  return BT::NodeStatus::SUCCESS;
}

CheckRobotPoseNode::CheckRobotPoseNode(const std::string& name,
                                       const BT::NodeConfiguration& config,
                                       NavigationContext* context)
    : BT::ConditionNode(name, config), context_(context) {}

BT::NodeStatus CheckRobotPoseNode::tick() {
  geometry_msgs::PoseStamped pose;
  if (!context_->currentPose(pose)) {
    setPoseFailure(context_, "odometry_or_tf_unavailable");
    MOTION_PLANNER_LOG_WARN_THROTTLE(
        2.0, "Robot pose unavailable: mission_id=%llu reason=%s",
        static_cast<unsigned long long>(context_->missionId()),
        context_->lastError().c_str());
    context_->publishState("WAITING_FOR_POSE", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  context_->clearFailure();
  return BT::NodeStatus::SUCCESS;
}

ComputePCTPathNode::ComputePCTPathNode(const std::string& name,
                                       const BT::NodeConfiguration& config,
                                       NavigationContext* context)
    : BT::StatefulActionNode(name, config), context_(context) {}

BT::NodeStatus ComputePCTPathNode::onStart() {
  if (!context_->globalPlanBudgetAvailable()) {
    context_->setFailure(FailureDomain::PCT_PLANNER,
                         FailureCode::GLOBAL_REPLAN_EXHAUSTED,
                         "global_replan_cycles_exhausted");
    MOTION_PLANNER_LOG_ERROR(
        "event=global_planning_finished mission_id=%llu route_id=%llu "
        "planning_attempt=%u request_id=%llu success=false "
        "reason_code=global_replan_cycles_exhausted duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        context_->planningAttempt(),
        static_cast<unsigned long long>(pct_request_id_),
        context_->missionDurationMs());
    context_->publishState("PLANNING_FAILED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::PoseStamped start;
  if (!context_->currentPose(start)) {
    setPoseFailure(context_, "odometry_or_tf_unavailable");
    MOTION_PLANNER_LOG_WARN_THROTTLE(
        2.0, "Cannot start PCT planning without robot pose: mission_id=%llu",
        static_cast<unsigned long long>(context_->missionId()));
    return BT::NodeStatus::FAILURE;
  }
  MOTION_PLANNER_LOG_DEBUG(
      "Waiting for PCT action server: mission_id=%llu timeout=%.1f",
      static_cast<unsigned long long>(context_->missionId()),
      context_->actionWaitTimeout());
  if (!context_->pctClient().waitForServer(
          ros::Duration(context_->actionWaitTimeout()))) {
    context_->setFailure(FailureDomain::PCT_PLANNER,
                         FailureCode::PCT_ACTION_UNAVAILABLE,
                         "pct_action_unavailable");
    MOTION_PLANNER_LOG_WARN(
        "PCT action server unavailable: mission_id=%llu timeout=%.1f",
        static_cast<unsigned long long>(context_->missionId()),
        context_->actionWaitTimeout());
    return BT::NodeStatus::FAILURE;
  }

  context_->setRequestStart(start);
  global_pct_planner::PlanPath3DGoal goal;
  goal.start = start;
  goal.goal = context_->goal();
  planning_trigger_ = context_->planningTrigger();
  planning_reason_ = normalizeLogToken(context_->planningReason(),
                                       "planner_recovery");
  planning_attempt_ = context_->planningAttempt() + 1;
  context_->setPctPlanningSnapshotVersion(0);
  goal.mission_id = context_->missionId();
  goal.planning_attempt = planning_attempt_;
  goal.trigger = planning_trigger_;
  goal.trigger_reason = planning_reason_;
  pct_request_id_ = 0;
  context_->setPlanningRequestId(0);
  context_->publishState("GLOBAL_PLANNING");
  MOTION_PLANNER_LOG_INFO(
      "event=global_planning_started mission_id=%llu route_id=%llu "
      "planning_attempt=%u request_id=%llu trigger=%s "
      "trigger_reason=%s start_frame=%s goal_frame=%s start_x=%.3f "
      "start_y=%.3f start_z=%.3f goal_x=%.3f goal_y=%.3f goal_z=%.3f",
      static_cast<unsigned long long>(context_->missionId()),
      static_cast<unsigned long long>(context_->routeId()), planning_attempt_,
      static_cast<unsigned long long>(pct_request_id_),
      planning_trigger_.c_str(), planning_reason_.c_str(),
      goal.start.header.frame_id.c_str(), goal.goal.header.frame_id.c_str(),
      goal.start.pose.position.x, goal.start.pose.position.y,
      goal.start.pose.position.z, goal.goal.pose.position.x,
      goal.goal.pose.position.y, goal.goal.pose.position.z);
  context_->pctClient().sendGoal(
      goal, NavigationContext::PctClient::SimpleDoneCallback(),
      NavigationContext::PctClient::SimpleActiveCallback(),
      boost::bind(&ComputePCTPathNode::pctFeedback, this,
                  boost::placeholders::_1));
  context_->recordGlobalPlanRequest();
  action_started_ = ros::Time::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ComputePCTPathNode::onRunning() {
  if ((ros::Time::now() - action_started_).toSec() >
      context_->actionExecutionTimeout()) {
    context_->pctClient().cancelGoal();
    context_->setFailure(FailureDomain::TIMEOUT,
                         FailureCode::ACTION_TIMEOUT, "pct_action_timeout");
    MOTION_PLANNER_LOG_ERROR(
        "event=global_planning_finished mission_id=%llu route_id=%llu "
        "planning_attempt=%u request_id=%llu success=false "
        "reason_code=pct_action_timeout timeout=%.1f duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        planning_attempt_, static_cast<unsigned long long>(pct_request_id_),
        context_->actionExecutionTimeout(),
        (ros::Time::now() - action_started_).toSec() * 1000.0);
    context_->publishState("TIMEOUT", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  const actionlib::SimpleClientGoalState state = context_->pctClient().getState();
  if (!state.isDone()) return BT::NodeStatus::RUNNING;

  const auto result = context_->pctClient().getResult();
  if (!result || result->status != global_pct_planner::PlanPath3DResult::SUCCESS) {
    const uint8_t status =
        result ? result->status
               : static_cast<uint8_t>(
                     global_pct_planner::PlanPath3DResult::INTERNAL_ERROR);
    const std::string reason =
        result && !result->reason_code.empty()
            ? result->reason_code
            : "no_pct_action_result";
    setPctFailure(context_, status, reason,
                  result ? result->message : "no_pct_action_result");
    MOTION_PLANNER_LOG_ERROR(
        "event=global_planning_finished mission_id=%llu route_id=%llu "
        "planning_attempt=%u request_id=%llu success=false status=%u "
        "reason_code=%s duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        planning_attempt_, static_cast<unsigned long long>(pct_request_id_),
        static_cast<unsigned int>(status), reason.c_str(),
        (ros::Time::now() - action_started_).toSec() * 1000.0);
    context_->publishState("PLANNING_FAILED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  if (result->path.poses.size() < 2) {
    context_->setFailure(FailureDomain::PCT_PLANNER,
                         FailureCode::RAW_PATH_TOO_SHORT,
                         "pct_raw_path_too_short");
    MOTION_PLANNER_LOG_ERROR(
        "event=global_planning_finished mission_id=%llu route_id=%llu "
        "planning_attempt=%u request_id=%llu success=false status=%u "
        "reason_code=pct_raw_path_too_short path_points=%zu duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()), planning_attempt_,
        static_cast<unsigned long long>(pct_request_id_),
        static_cast<unsigned int>(
            global_pct_planner::PlanPath3DResult::NO_PATH),
        result->path.poses.size(),
        (ros::Time::now() - action_started_).toSec() * 1000.0);
    context_->publishState("PLANNING_FAILED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }

  context_->setRawPath(result->path);
  if (result->has_snapped_goal)
    context_->setTerminalGoal(result->snapped_goal);
  else
    context_->setTerminalGoal(result->path.poses.back());
  context_->clearFailure();
  MOTION_PLANNER_LOG_INFO(
      "event=global_planning_finished mission_id=%llu route_id=%llu "
      "planning_attempt=%u request_id=%llu success=true status=%u "
      "reason_code=ok path_points=%zu snapped_goal=%s duration_ms=%.3f",
      static_cast<unsigned long long>(context_->missionId()),
      static_cast<unsigned long long>(context_->routeId()), planning_attempt_,
      static_cast<unsigned long long>(pct_request_id_),
      static_cast<unsigned int>(global_pct_planner::PlanPath3DResult::SUCCESS),
      result->path.poses.size(), result->has_snapped_goal ? "true" : "false",
      (ros::Time::now() - action_started_).toSec() * 1000.0);
  return BT::NodeStatus::SUCCESS;
}

void ComputePCTPathNode::pctFeedback(
    const global_pct_planner::PlanPath3DFeedbackConstPtr& feedback) {
  if (!feedback) return;
  pct_request_id_ = feedback->request_id;
  context_->setPlanningRequestId(pct_request_id_);
  context_->setPctPlanningSnapshotVersion(feedback->dynamic_snapshot_version);
}

void ComputePCTPathNode::onHalted() { context_->pctClient().cancelGoal(); }

ProcessReferencePathNode::ProcessReferencePathNode(
    const std::string& name, const BT::NodeConfiguration& config,
    NavigationContext* context)
    : BT::SyncActionNode(name, config), context_(context) {}

BT::NodeStatus ProcessReferencePathNode::tick() {
  std::string error;
  if (!context_->processReferencePath(error)) {
    setPathFailure(context_, error);
    context_->publishState("PATH_REJECTED", error);
    return BT::NodeStatus::FAILURE;
  }
  context_->clearFailure();
  context_->publishState("PATH_READY");
  MOTION_PLANNER_LOG_INFO(
      "Reference path ready: mission_id=%llu points=%zu frame=%s",
      static_cast<unsigned long long>(context_->missionId()),
      context_->processedPath().poses.size(),
      context_->processedPath().header.frame_id.c_str());
  return BT::NodeStatus::SUCCESS;
}

ValidateReferencePathNode::ValidateReferencePathNode(
    const std::string& name, const BT::NodeConfiguration& config,
    NavigationContext* context)
    : BT::ConditionNode(name, config), context_(context) {}

BT::NodeStatus ValidateReferencePathNode::tick() {
  const auto& path = context_->processedPath();
  if (path.header.frame_id != context_->navigationFrame() ||
      path.poses.size() < 2) {
    context_->setFailure(FailureDomain::REFERENCE_PATH,
                         FailureCode::PROCESSED_PATH_INVALID,
                         "processed_path_invalid");
    context_->publishState("PATH_REJECTED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  context_->clearFailure();
  return BT::NodeStatus::SUCCESS;
}

FollowReferencePathNode::FollowReferencePathNode(
    const std::string& name, const BT::NodeConfiguration& config,
    NavigationContext* context)
    : BT::StatefulActionNode(name, config), context_(context) {}

BT::NodeStatus FollowReferencePathNode::onStart() {
  if (!context_->scanClient().waitForServer(
          ros::Duration(context_->actionWaitTimeout()))) {
    context_->setFailure(FailureDomain::SCAN_CONTROLLER,
                         FailureCode::SCAN_ACTION_UNAVAILABLE,
                         "scan_follow_action_unavailable");
    MOTION_PLANNER_LOG_WARN(
        "SCAN action server unavailable: mission_id=%llu timeout=%.1f",
        static_cast<unsigned long long>(context_->missionId()),
        context_->actionWaitTimeout());
    return BT::NodeStatus::FAILURE;
  }
  navigation_msgs::FollowReferencePathGoal goal;
  goal.mission_id = context_->missionId();
  goal.request_id = context_->planningRequestId();
  goal.planning_attempt = context_->planningAttempt();
  goal.trigger = context_->planningTrigger();
  goal.trigger_reason = context_->planningReason();
  request_id_ = goal.request_id;
  planning_attempt_ = goal.planning_attempt;
  planning_trigger_ = goal.trigger;
  planning_reason_ = goal.trigger_reason;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_wall_ = ros::WallTime::now();
    received_feedback_ = false;
  }
  const uint64_t route_id = context_->nextRouteId();
  goal.route_id = route_id;
  goal.path = context_->processedPath();
  context_->publishState("LOCAL_EXECUTING");
  MOTION_PLANNER_LOG_INFO(
      "event=local_execution_started mission_id=%llu route_id=%llu "
      "request_id=%llu planning_attempt=%u trigger=%s trigger_reason=%s "
      "path_points=%zu frame=%s",
      static_cast<unsigned long long>(goal.mission_id),
      static_cast<unsigned long long>(route_id),
      static_cast<unsigned long long>(goal.request_id), goal.planning_attempt,
      goal.trigger.c_str(), goal.trigger_reason.c_str(),
      goal.path.poses.size(),
      goal.path.header.frame_id.c_str());
  context_->scanClient().sendGoal(
      goal, NavigationContext::ScanClient::SimpleDoneCallback(),
      NavigationContext::ScanClient::SimpleActiveCallback(),
      boost::bind(&FollowReferencePathNode::scanFeedback, this,
                  boost::placeholders::_1));
  action_started_ = ros::Time::now();
  action_started_wall_ = ros::WallTime::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowReferencePathNode::onRunning() {
  if ((ros::Time::now() - action_started_).toSec() >
      context_->actionExecutionTimeout()) {
    context_->scanClient().cancelGoal();
    context_->setFailure(FailureDomain::TIMEOUT,
                         FailureCode::ACTION_TIMEOUT, "scan_action_timeout");
    MOTION_PLANNER_LOG_ERROR(
        "event=local_execution_finished mission_id=%llu route_id=%llu "
      "success=false reason_code=scan_action_timeout timeout=%.1f "
      "request_id=%llu planning_attempt=%u trigger=%s trigger_reason=%s "
      "duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        context_->actionExecutionTimeout(),
        static_cast<unsigned long long>(request_id_), planning_attempt_,
        planning_trigger_.c_str(), planning_reason_.c_str(),
        (ros::Time::now() - action_started_).toSec() * 1000.0);
    context_->publishState("TIMEOUT", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  const actionlib::SimpleClientGoalState action_state =
      context_->scanClient().getState();
  if (!action_state.isDone()) {
    bool feedback_stale = false;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      const ros::WallTime reference = received_feedback_
                                          ? last_feedback_wall_
                                          : action_started_wall_;
      feedback_stale = context_->followFeedbackTimeoutSec() > 0.0 &&
          (ros::WallTime::now() - reference).toSec() >
          context_->followFeedbackTimeoutSec();
    }
    if (!feedback_stale) return BT::NodeStatus::RUNNING;

    context_->scanClient().cancelGoal();
    context_->setFailure(FailureDomain::TIMEOUT, FailureCode::ACTION_TIMEOUT,
                         "scan_feedback_timeout");
    MOTION_PLANNER_LOG_ERROR(
        "event=local_execution_finished mission_id=%llu route_id=%llu "
        "request_id=%llu success=false reason_code=scan_feedback_timeout "
        "feedback_timeout_sec=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        static_cast<unsigned long long>(request_id_),
        context_->followFeedbackTimeoutSec());
    context_->publishState("TIMEOUT", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }

  const auto result = context_->scanClient().getResult();
  if (!result) {
    context_->setFailure(FailureDomain::SCAN_CONTROLLER,
                         FailureCode::SCAN_FAILED, "no_scan_action_result");
    MOTION_PLANNER_LOG_ERROR(
        "event=local_execution_finished mission_id=%llu route_id=%llu "
        "request_id=%llu planning_attempt=%u trigger=%s "
        "trigger_reason=%s success=false reason_code=no_scan_action_result "
        "duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        static_cast<unsigned long long>(request_id_), planning_attempt_,
        planning_trigger_.c_str(), planning_reason_.c_str(),
        (ros::Time::now() - action_started_).toSec() * 1000.0);
    return BT::NodeStatus::FAILURE;
  }
  if (result->result == navigation_msgs::FollowReferencePathResult::SUCCEEDED) {
    context_->setExecutionResult(result->execution_goal, result->final_pose,
                                 result->remaining_distance,
                                 result->endpoint_truncated);
    context_->clearFailure();
    MOTION_PLANNER_LOG_INFO(
        "event=local_execution_finished mission_id=%llu route_id=%llu "
        "request_id=%llu planning_attempt=%u trigger=%s trigger_reason=%s "
        "success=true result=SUCCEEDED duration_ms=%.3f",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        static_cast<unsigned long long>(request_id_), planning_attempt_,
        planning_trigger_.c_str(), planning_reason_.c_str(),
        (ros::Time::now() - action_started_).toSec() * 1000.0);
    return BT::NodeStatus::SUCCESS;
  }

  const std::string message = result->message.empty() ? "scan_execution_failed"
                                                      : result->message;
  const std::string reason_code = result->reason_code.empty()
                                      ? "scan_execution_failed"
                                      : result->reason_code;
  FailureCode code = FailureCode::SCAN_FAILED;
  FailureDomain domain = FailureDomain::SCAN_CONTROLLER;
  std::string navigation_state = "EXECUTION_FAILED";
  switch (result->result) {
    case navigation_msgs::FollowReferencePathResult::REJECTED:
      code = FailureCode::SCAN_REJECTED;
      navigation_state = "PATH_REJECTED";
      break;
    case navigation_msgs::FollowReferencePathResult::CANCELED:
      domain = FailureDomain::CANCELED;
      code = FailureCode::SCAN_CANCELED;
      navigation_state = "CANCELED";
      break;
    case navigation_msgs::FollowReferencePathResult::EMERGENCY_STOPPED:
      domain = FailureDomain::EMERGENCY;
      code = FailureCode::SCAN_EMERGENCY_STOPPED;
      navigation_state = "EMERGENCY_STOPPED";
      break;
    default:
      break;
  }
  if (result->result == navigation_msgs::FollowReferencePathResult::FAILED) {
    if (reason_code == "local_replan_exhausted")
      code = FailureCode::SCAN_LOCAL_REPLAN_EXHAUSTED;
    else if (reason_code == "progress_stalled")
      code = FailureCode::SCAN_PROGRESS_STALLED;
    else if (reason_code == "trajectory_deviation")
      code = FailureCode::SCAN_PROGRESS_STALLED;
    else if (reason_code == "goal_not_reached")
      code = FailureCode::SCAN_GOAL_NOT_REACHED;
    else if (reason_code == "endpoint_truncated")
      code = FailureCode::SCAN_ENDPOINT_TRUNCATED;
  }
  context_->setFailure(domain, code, message);
  MOTION_PLANNER_LOG_WARN(
      "event=local_execution_finished mission_id=%llu route_id=%llu "
      "request_id=%llu planning_attempt=%u trigger=%s trigger_reason=%s "
      "success=false result=%u reason_code=%s duration_ms=%.3f",
      static_cast<unsigned long long>(context_->missionId()),
      static_cast<unsigned long long>(context_->routeId()),
      static_cast<unsigned long long>(request_id_), planning_attempt_,
      planning_trigger_.c_str(), planning_reason_.c_str(),
      static_cast<unsigned int>(result->result),
      reason_code.c_str(),
      (ros::Time::now() - action_started_).toSec() * 1000.0);
  context_->publishState(navigation_state, context_->lastError());
  return BT::NodeStatus::FAILURE;
}

void FollowReferencePathNode::scanFeedback(
    const navigation_msgs::FollowReferencePathFeedbackConstPtr& feedback) {
  if (!feedback) return;
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  received_feedback_ = true;
  last_feedback_wall_ = ros::WallTime::now();
}

void FollowReferencePathNode::onHalted() {
  context_->scanClient().cancelGoal();
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  received_feedback_ = false;
}

NavigationRecoveryNode::NavigationRecoveryNode(
    const std::string& name, const BT::NodeConfiguration& config)
    : BT::ControlNode(name, config) {}

void NavigationRecoveryNode::halt() {
  BT::ControlNode::halt();
  current_child_idx_ = 0;
}

BT::NodeStatus NavigationRecoveryNode::tick() {
  if (childrenCount() != 2) {
    throw BT::LogicError("NavigationRecovery requires exactly two children");
  }

  setStatus(BT::NodeStatus::RUNNING);
  while (current_child_idx_ < 2) {
    const BT::NodeStatus child_status =
        children_nodes_[current_child_idx_]->executeTick();
    if (current_child_idx_ == 0) {
      if (child_status == BT::NodeStatus::SUCCESS) {
        haltChild(1);
        halt();
        return BT::NodeStatus::SUCCESS;
      }
      if (child_status == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      haltChild(0);
      current_child_idx_ = 1;
      continue;
    }

    if (child_status == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    if (child_status == BT::NodeStatus::SUCCESS) {
      haltChild(1);
      current_child_idx_ = 0;
      return BT::NodeStatus::RUNNING;
    }
    halt();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

RecoveryDispatcherNode::RecoveryDispatcherNode(
    const std::string& name, const BT::NodeConfiguration& config,
    NavigationContext* context)
    : BT::StatefulActionNode(name, config), context_(context) {}

BT::NodeStatus RecoveryDispatcherNode::onStart() {
  recovery_failure_domain_ = context_->failureDomain();
  recovery_failure_code_ = context_->failureCode();
  recovery_action_ = recoveryActionFor(recovery_failure_domain_,
                                       recovery_failure_code_);
  const std::string reason = recoveryReasonCode(recovery_failure_domain_,
                                                recovery_failure_code_);

  MOTION_PLANNER_LOG_WARN(
      "event=recovery_evaluation mission_id=%llu route_id=%llu "
      "planning_attempt=%u action=%u reason_code=%s",
      static_cast<unsigned long long>(context_->missionId()),
      static_cast<unsigned long long>(context_->routeId()),
      context_->planningAttempt(), static_cast<unsigned int>(recovery_action_),
      reason.c_str());

  waiting_for_environment_ = false;
  waiting_for_dynamic_update_ = false;
  if (recovery_action_ == RecoveryAction::TERMINAL)
    return BT::NodeStatus::FAILURE;

  if (recovery_action_ == RecoveryAction::WAIT_FOR_ENVIRONMENT) {
    if (!context_->reserveEnvironmentWait()) {
      context_->setFailure(FailureDomain::TIMEOUT,
                           FailureCode::ENVIRONMENT_WAIT_EXHAUSTED,
                           "environment_wait_retries_exhausted");
      context_->publishState("FAILED", context_->lastError());
      return BT::NodeStatus::FAILURE;
    }
    waiting_for_environment_ = true;
    recovery_deadline_ = ros::WallTime::now() + ros::WallDuration(
        context_->environmentWaitTimeoutSec());
    context_->publishState("WAITING_FOR_ENVIRONMENT", reason);
    return onRunning();
  }

  const bool emergency_recovery =
      recovery_failure_domain_ == FailureDomain::EMERGENCY &&
      recovery_failure_code_ == FailureCode::SCAN_EMERGENCY_STOPPED;
  const bool scan_recovery = emergency_recovery ||
      recovery_failure_domain_ == FailureDomain::SCAN_CONTROLLER;
  if (emergency_recovery && !context_->reserveEmergencyRecovery()) {
    context_->setFailure(FailureDomain::EMERGENCY,
                         FailureCode::EMERGENCY_RECOVERY_EXHAUSTED,
                         "emergency_recovery_attempts_exhausted");
    context_->publishState("EMERGENCY_STOPPED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }

  const std::string trigger = scan_recovery ? "local_replan"
                                             : "planner_recovery";
  context_->prepareNextPlanningContext(trigger, reason);
  context_->clearCurrentRoute();
  recovery_action_ = recoveryActionFor(recovery_failure_domain_,
                                       recovery_failure_code_);
  waiting_for_dynamic_update_ =
      recovery_action_ == RecoveryAction::RETRY_GLOBAL_AFTER_DYNAMIC_UPDATE &&
      context_->requireDynamicUpdateAfterRecovery();
  const bool waiting_on_pct_snapshot =
      recovery_failure_domain_ == FailureDomain::PCT_PLANNER &&
      (recovery_failure_code_ == FailureCode::PCT_DYNAMIC_BLOCKED ||
       recovery_failure_code_ == FailureCode::PCT_DYNAMIC_SNAPSHOT_UNSTABLE);
  dynamic_version_at_start_ = waiting_on_pct_snapshot
                                   ? context_->pctPlanningSnapshotVersion()
                                   : context_->dynamicLayerVersion();
  if (scan_recovery || waiting_for_dynamic_update_) {
    recovery_ready_at_ = ros::WallTime::now() +
                         ros::WallDuration(context_->emergencyRecoveryWaitSec());
    recovery_deadline_ = ros::WallTime::now() + ros::WallDuration(
        context_->emergencyRecoveryWaitSec() +
        context_->dynamicUpdateTimeoutSec());
    context_->publishState(waiting_for_dynamic_update_ ? "RECOVERY_WAITING_FOR_PERCEPTION"
                                                       : "RECOVERY_WAITING",
                           reason);
    MOTION_PLANNER_LOG_INFO(
        "event=recovery_wait_started mission_id=%llu route_id=%llu "
        "planning_attempt=%u trigger=%s trigger_reason=%s wait_sec=%.3f "
        "dynamic_snapshot_version=%llu",
        static_cast<unsigned long long>(context_->missionId()),
        static_cast<unsigned long long>(context_->routeId()),
        context_->planningAttempt() + 1, trigger.c_str(), reason.c_str(),
        context_->emergencyRecoveryWaitSec(),
        static_cast<unsigned long long>(dynamic_version_at_start_));
  }
  return onRunning();
}

BT::NodeStatus RecoveryDispatcherNode::onRunning() {
  if (context_->missionDeadlineExceeded()) {
    context_->setFailure(FailureDomain::TIMEOUT, FailureCode::MISSION_DEADLINE,
                         "mission_deadline_exceeded");
    context_->publishState("TIMEOUT", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }

  if (waiting_for_environment_) {
    bool ready = false;
    switch (recovery_failure_code_) {
      case FailureCode::GOAL_TRANSFORM_FAILED: {
        geometry_msgs::PoseStamped transformed;
        ready = context_->transformPose(context_->rawGoal(), transformed);
        break;
      }
      case FailureCode::ODOM_TF_UNAVAILABLE: {
        geometry_msgs::PoseStamped pose;
        ready = context_->currentPose(pose);
        break;
      }
      case FailureCode::PCT_ACTION_UNAVAILABLE:
        ready = context_->pctClient().isServerConnected();
        break;
      case FailureCode::SCAN_ACTION_UNAVAILABLE:
        ready = context_->scanClient().isServerConnected();
        break;
      default:
        break;
    }
    if (ready) {
      waiting_for_environment_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    if (ros::WallTime::now() >= recovery_deadline_) {
      context_->setFailure(FailureDomain::TIMEOUT,
                           FailureCode::ENVIRONMENT_WAIT_EXHAUSTED,
                           "environment_wait_timeout");
      context_->publishState("FAILED", context_->lastError());
      return BT::NodeStatus::FAILURE;
    }
    context_->publishState("WAITING_FOR_ENVIRONMENT", context_->lastError());
    return BT::NodeStatus::RUNNING;
  }

  if (ros::WallTime::now() < recovery_ready_at_)
    return BT::NodeStatus::RUNNING;

  if (waiting_for_dynamic_update_) {
    if (ros::WallTime::now() >= recovery_deadline_) {
      context_->setFailure(FailureDomain::PCT_PLANNER,
                           FailureCode::DYNAMIC_PERCEPTION_UNAVAILABLE,
                           "dynamic_perception_refresh_timeout");
      context_->publishState("PLANNING_FAILED", context_->lastError());
      return BT::NodeStatus::FAILURE;
    }
    if (!context_->dynamicLayerEnabledKnown())
      return BT::NodeStatus::RUNNING;
    if (context_->dynamicLayerEnabled()) {
      const bool refreshed = context_->dynamicLayerStateKnown() &&
                             context_->dynamicLayerHealthy() &&
                             context_->dynamicLayerVersion() >
                                 dynamic_version_at_start_;
      if (!refreshed) {
        context_->publishState("RECOVERY_WAITING_FOR_PERCEPTION",
                               "awaiting_fresh_dynamic_snapshot");
        return BT::NodeStatus::RUNNING;
      }
    }
    waiting_for_dynamic_update_ = false;
  }

  context_->publishState("GLOBAL_REPLANNING", context_->planningReason());
  MOTION_PLANNER_LOG_INFO(
      "event=global_replanning_started mission_id=%llu route_id=%llu "
      "planning_attempt=%u trigger=%s trigger_reason=%s replan_count=%u",
      static_cast<unsigned long long>(context_->missionId()),
      static_cast<unsigned long long>(context_->routeId()),
      context_->planningAttempt() + 1, context_->planningTrigger().c_str(),
      context_->planningReason().c_str(), context_->replanCount() + 1);
  return BT::NodeStatus::SUCCESS;
}

void RecoveryDispatcherNode::onHalted() {
  waiting_for_environment_ = false;
  waiting_for_dynamic_update_ = false;
  recovery_action_ = RecoveryAction::TERMINAL;
}

CheckGoalReachedNode::CheckGoalReachedNode(const std::string& name,
                                           const BT::NodeConfiguration& config,
                                           NavigationContext* context)
    : BT::ConditionNode(name, config), context_(context) {}

BT::NodeStatus CheckGoalReachedNode::tick() {
  if (!context_->goalReached()) {
    // The first CheckGoalReached in the fallback is an expected false branch:
    // it selects path execution and must not look like an execution error.
    if (context_->processedPath().poses.empty()) {
      context_->clearFailure();
      MOTION_PLANNER_LOG_DEBUG(
          "event=goal_not_reached mission_id=%llu status=continue",
          static_cast<unsigned long long>(context_->missionId()));
      context_->publishState("GOAL_NOT_REACHED");
      return BT::NodeStatus::FAILURE;
    }
    context_->setFailure(FailureDomain::SCAN_CONTROLLER,
                         FailureCode::SCAN_GOAL_NOT_REACHED,
                         "goal_not_reached_after_route");
    MOTION_PLANNER_LOG_WARN(
        "event=goal_not_reached mission_id=%llu status=failed "
        "reason_code=goal_not_reached_after_route",
        static_cast<unsigned long long>(context_->missionId()));
    context_->publishState("GOAL_NOT_REACHED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  context_->clearFailure();
  context_->publishState("COMPLETED");
  return BT::NodeStatus::SUCCESS;
}

MonitorProgressNode::MonitorProgressNode(const std::string& name,
                                         const BT::NodeConfiguration& config,
                                         NavigationContext* context)
    : BT::SyncActionNode(name, config), context_(context) {}

BT::NodeStatus MonitorProgressNode::tick() {
  if (context_->executionGoal().header.frame_id.empty() ||
      context_->executionRemainingDistance() > context_->goalReachedDistance()) {
    context_->setFailure(FailureDomain::SCAN_CONTROLLER,
                         FailureCode::SCAN_GOAL_NOT_REACHED,
                         "scan_reported_success_outside_goal_tolerance");
    context_->publishState("EXECUTION_FAILED", context_->lastError());
    return BT::NodeStatus::FAILURE;
  }
  context_->publishState("PROGRESS_VALIDATED");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace navigation_supervisor
