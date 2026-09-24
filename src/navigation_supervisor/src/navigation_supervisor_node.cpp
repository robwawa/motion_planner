#include <cmath>
#include <cctype>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <stdexcept>

#include <actionlib/server/simple_action_server.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <boost/bind/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <navigation_msgs/NavigateToPoseAction.h>
#include <motion_planner_log/logging.h>
#include <ros/ros.h>

#include "navigation_supervisor/bt_nodes.h"
#include "navigation_supervisor/navigation_context.h"

namespace navigation_supervisor {

namespace {

std::string normalizeLogToken(const std::string& value,
                              const std::string& fallback) {
  if (value.empty()) return fallback;
  std::string output;
  output.reserve(value.size());
  for (const char character : value) {
    const unsigned char byte = static_cast<unsigned char>(character);
    if (std::isalnum(byte) || character == '_' || character == '-')
      output.push_back(static_cast<char>(std::tolower(byte)));
    else
      output.push_back('_');
  }
  return output.empty() ? fallback : output;
}

}  // namespace

class NavigationSupervisor {
 public:
  using ActionServer =
      actionlib::SimpleActionServer<navigation_msgs::NavigateToPoseAction>;
  using ActionClient =
      actionlib::SimpleActionClient<navigation_msgs::NavigateToPoseAction>;

  NavigationSupervisor(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(std::move(nh)),
        private_nh_(std::move(private_nh)),
        context_(nh_, private_nh_),
        action_server_(private_nh_, "/navigation/navigate_to_pose",
                       boost::bind(&NavigationSupervisor::execute, this,
                                   boost::placeholders::_1),
                       false),
        self_client_(nh_, "/navigation/navigate_to_pose", true) {
    private_nh_.param<std::string>(
        "goal_topic", goal_topic_, std::string("/goal_pose_3d"));
    private_nh_.param<std::string>(
        "rviz_goal_topic", rviz_goal_topic_,
        std::string("/move_base_simple/goal"));
    private_nh_.param("default_goal_z", default_goal_z_, 0.4);
    private_nh_.param<std::string>("tree_file", tree_file_, std::string());
    if (tree_file_.empty()) {
      MOTION_PLANNER_LOG_FATAL(
          "Required private parameter ~tree_file is empty");
      throw std::runtime_error("tree_file is empty");
    }

    goal_sub_ = nh_.subscribe(goal_topic_, 1,
                              &NavigationSupervisor::goalCallback, this);
    rviz_goal_sub_ = nh_.subscribe(rviz_goal_topic_, 1,
                                   &NavigationSupervisor::rvizGoalCallback,
                                   this);
    dispatch_timer_ = nh_.createTimer(
        ros::Duration(0.1), &NavigationSupervisor::dispatchTopicGoal, this);
    action_server_.start();
    MOTION_PLANNER_LOG_INFO(
        "Ready: tree=%s goal_topic=%s rviz_goal_topic=%s frame=%s "
        "action_wait_timeout=%.1f action_execution_timeout=%.1f "
        "max_replan_cycles=%d",
        tree_file_.c_str(), goal_topic_.c_str(), rviz_goal_topic_.c_str(),
        context_.navigationFrame().c_str(), context_.actionWaitTimeout(),
        context_.actionExecutionTimeout(), context_.maxReplanCycles());
  }

 private:
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    if (!message) return;
    {
      std::lock_guard<std::mutex> lock(topic_mutex_);
      pending_goal_ = *message;
      have_pending_goal_ = true;
    }
    MOTION_PLANNER_LOG_INFO(
        "event=mission_received mission_id=0 route_id=0 frame=%s x=%.3f "
        "y=%.3f z=%.3f source=topic",
        message->header.frame_id.c_str(), message->pose.position.x,
        message->pose.position.y, message->pose.position.z);
  }

  void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr& message) {
    if (!message) return;
    geometry_msgs::PoseStamped goal = *message;
    bool height_adjusted = false;
    if (!std::isfinite(goal.pose.position.z) ||
        std::abs(goal.pose.position.z) < 1e-6) {
      geometry_msgs::PoseStamped current;
      if (context_.currentPose(current))
        goal.pose.position.z = current.pose.position.z;
      else
        goal.pose.position.z = default_goal_z_;
      height_adjusted = true;
    }
    {
      std::lock_guard<std::mutex> lock(topic_mutex_);
      pending_goal_ = goal;
      have_pending_goal_ = true;
    }
    if (height_adjusted) {
      MOTION_PLANNER_LOG_WARN(
          "event=mission_received_adjusted source=rviz reason_code=goal_z_adjusted "
          "z=%.3f frame=%s",
          goal.pose.position.z, goal.header.frame_id.c_str());
    } else {
      MOTION_PLANNER_LOG_INFO(
          "event=mission_received mission_id=0 route_id=0 source=rviz "
          "frame=%s x=%.3f y=%.3f z=%.3f",
          goal.header.frame_id.c_str(), goal.pose.position.x,
          goal.pose.position.y, goal.pose.position.z);
    }
  }

  void dispatchTopicGoal(const ros::TimerEvent&) {
    geometry_msgs::PoseStamped goal;
    {
      std::lock_guard<std::mutex> lock(topic_mutex_);
      if (!have_pending_goal_)
        return;
      if (!self_client_.isServerConnected()) {
        MOTION_PLANNER_LOG_WARN_THROTTLE(
            5.0, "Navigation action server is not connected; keep pending goal");
        return;
      }
      goal = pending_goal_;
      have_pending_goal_ = false;
    }
    navigation_msgs::NavigateToPoseGoal action_goal;
    action_goal.goal = goal;
    self_client_.sendGoal(action_goal);
    MOTION_PLANNER_LOG_INFO(
        "event=mission_dispatched mission_id=0 route_id=0 source=topic");
  }

  void registerNodes(BT::BehaviorTreeFactory& factory) {
    registerContextNode<ValidateGoalNode>(factory, "ValidateGoal");
    registerContextNode<TransformGoalNode>(factory, "TransformGoal");
    registerContextNode<CheckRobotPoseNode>(factory, "CheckRobotPose");
    registerContextNode<ComputePCTPathNode>(factory, "ComputePCTPath");
    registerContextNode<ProcessReferencePathNode>(factory, "ProcessReferencePath");
    registerContextNode<ValidateReferencePathNode>(factory, "ValidateReferencePath");
    registerContextNode<FollowReferencePathNode>(factory, "FollowReferencePath");
    registerContextNode<CheckGoalReachedNode>(factory, "CheckGoalReached");
    registerContextNode<MonitorProgressNode>(factory, "MonitorProgress");
    factory.registerBuilder<NavigationRecoveryNode>(
        "NavigationRecovery",
        [](const std::string& name, const BT::NodeConfiguration& config) {
          return std::unique_ptr<BT::TreeNode>(
              new NavigationRecoveryNode(name, config));
        });
    registerContextNode<RecoveryDispatcherNode>(factory,
                                                "RecoveryDispatcher");
  }

  template <typename NodeT>
  void registerContextNode(BT::BehaviorTreeFactory& factory,
                           const std::string& id) {
    factory.registerBuilder<NodeT>(
        id, [this](const std::string& name,
                   const BT::NodeConfiguration& config) {
          return std::unique_ptr<BT::TreeNode>(
              new NodeT(name, config, &context_));
        });
  }

  void execute(const navigation_msgs::NavigateToPoseGoalConstPtr& goal) {
    if (!goal) return;
    context_.beginMission(goal->goal);
    MOTION_PLANNER_LOG_INFO(
        "event=mission_started mission_id=%llu route_id=%llu frame=%s "
        "x=%.3f y=%.3f z=%.3f",
        static_cast<unsigned long long>(context_.missionId()),
        static_cast<unsigned long long>(context_.routeId()),
        goal->goal.header.frame_id.c_str(), goal->goal.pose.position.x,
        goal->goal.pose.position.y, goal->goal.pose.position.z);

    BT::BehaviorTreeFactory factory;
    registerNodes(factory);
    auto blackboard = BT::Blackboard::create();
    BT::Tree tree = factory.createTreeFromFile(tree_file_, blackboard);

    ros::Rate rate(20.0);
    while (ros::ok()) {
      if (action_server_.isPreemptRequested()) {
        context_.cancelActiveActions();
        tree.haltTree();
        navigation_msgs::NavigateToPoseResult result;
        result.result = navigation_msgs::NavigateToPoseResult::CANCELED;
        result.message = "navigation canceled";
        action_server_.setPreempted(result, result.message);
        context_.publishState("CANCELED", result.message);
        MOTION_PLANNER_LOG_WARN(
            "event=mission_canceled mission_id=%llu route_id=%llu "
            "reason_code=action_preempted duration_ms=%.3f",
            static_cast<unsigned long long>(context_.missionId()),
            static_cast<unsigned long long>(context_.routeId()),
            context_.missionDurationMs());
        return;
      }

      if (context_.missionDeadlineExceeded()) {
        context_.cancelActiveActions();
        tree.haltTree();
        context_.setFailure(FailureDomain::TIMEOUT, FailureCode::MISSION_DEADLINE,
                            "mission_deadline_exceeded");
        navigation_msgs::NavigateToPoseResult result;
        result.result = navigation_msgs::NavigateToPoseResult::TIMEOUT;
        result.message = context_.lastError();
        action_server_.setAborted(result, result.message);
        context_.publishState("TIMEOUT", result.message);
        MOTION_PLANNER_LOG_ERROR(
            "event=mission_failed mission_id=%llu route_id=%llu "
            "result=TIMEOUT reason_code=mission_deadline_exceeded "
            "replan_count=%u duration_ms=%.3f",
            static_cast<unsigned long long>(context_.missionId()),
            static_cast<unsigned long long>(context_.routeId()),
            context_.replanCount(), context_.missionDurationMs());
        return;
      }

      const BT::NodeStatus status = tree.tickRoot();
      navigation_msgs::NavigateToPoseFeedback feedback;
      feedback.mission_id = context_.missionId();
      feedback.route_id = context_.routeId();
      feedback.state = context_.lastError().empty() ? "RUNNING"
                                                     : context_.lastError();
      feedback.distance_remaining = context_.distanceToGoal();
      feedback.replan_count = context_.replanCount();
      action_server_.publishFeedback(feedback);

      if (status == BT::NodeStatus::SUCCESS) {
        navigation_msgs::NavigateToPoseResult result;
        result.result = navigation_msgs::NavigateToPoseResult::SUCCESS;
        result.message = "navigation completed";
        action_server_.setSucceeded(result, result.message);
        MOTION_PLANNER_LOG_INFO(
            "event=mission_completed mission_id=%llu route_id=%llu "
            "replan_count=%u duration_ms=%.3f",
            static_cast<unsigned long long>(context_.missionId()),
            static_cast<unsigned long long>(context_.routeId()),
            context_.replanCount(), context_.missionDurationMs());
        return;
      }
      if (status == BT::NodeStatus::FAILURE) {
        navigation_msgs::NavigateToPoseResult result;
        result.result = classifyFailure();
        result.message = context_.lastError().empty()
                             ? "navigation failed"
                             : context_.lastError();
        action_server_.setAborted(result, result.message);
        context_.publishState("FAILED", result.message);
        MOTION_PLANNER_LOG_ERROR(
            "event=mission_failed mission_id=%llu route_id=%llu result=%u "
            "reason_code=%s replan_count=%u duration_ms=%.3f",
            static_cast<unsigned long long>(context_.missionId()),
            static_cast<unsigned long long>(context_.routeId()),
            static_cast<unsigned int>(result.result),
            normalizeLogToken(result.message, "navigation_failed").c_str(),
            context_.replanCount(),
            context_.missionDurationMs());
        return;
      }
      rate.sleep();
    }
  }

  uint8_t classifyFailure() const {
    switch (context_.failureDomain()) {
      case FailureDomain::CANCELED:
        return navigation_msgs::NavigateToPoseResult::CANCELED;
      case FailureDomain::TIMEOUT:
        return navigation_msgs::NavigateToPoseResult::TIMEOUT;
      case FailureDomain::EMERGENCY:
        return navigation_msgs::NavigateToPoseResult::EMERGENCY_STOPPED;
      case FailureDomain::GOAL:
      case FailureDomain::REFERENCE_PATH:
        return navigation_msgs::NavigateToPoseResult::ROUTE_REJECTED;
      case FailureDomain::SCAN_CONTROLLER:
        if (context_.failureCode() == FailureCode::SCAN_REJECTED)
          return navigation_msgs::NavigateToPoseResult::ROUTE_REJECTED;
        return navigation_msgs::NavigateToPoseResult::EXECUTION_FAILED;
      case FailureDomain::PCT_PLANNER:
        if (context_.failureCode() == FailureCode::PCT_INVALID_REQUEST ||
            context_.failureCode() == FailureCode::PCT_FRAME_MISMATCH)
          return navigation_msgs::NavigateToPoseResult::ROUTE_REJECTED;
        return navigation_msgs::NavigateToPoseResult::PLANNING_FAILED;
      case FailureDomain::TF:
      case FailureDomain::NONE:
      default:
        return navigation_msgs::NavigateToPoseResult::PLANNING_FAILED;
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  NavigationContext context_;
  ActionServer action_server_;
  ActionClient self_client_;
  ros::Subscriber goal_sub_;
  ros::Subscriber rviz_goal_sub_;
  ros::Timer dispatch_timer_;
  std::mutex topic_mutex_;
  geometry_msgs::PoseStamped pending_goal_;
  bool have_pending_goal_{false};
  std::string goal_topic_;
  std::string rviz_goal_topic_;
  std::string tree_file_;
  double default_goal_z_{0.4};
};

}  // namespace navigation_supervisor

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_supervisor");
  motion_planner_log::initialize("navigation_supervisor", "navigation_supervisor", argv[0]);
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  try {
    navigation_supervisor::NavigationSupervisor supervisor(nh, private_nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception& exception) {
    MOTION_PLANNER_LOG_FATAL("Failed to start navigation supervisor: %s",
                             exception.what());
    return 1;
  }
  MOTION_PLANNER_LOG_INFO("Navigation supervisor shutting down");
  return 0;
}
