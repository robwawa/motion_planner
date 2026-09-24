#pragma once

#include <cstddef>
#include <mutex>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/control_node.h>

#include <ros/time.h>

#include "navigation_supervisor/navigation_context.h"

namespace navigation_supervisor {

class ValidateGoalNode : public BT::SyncActionNode {
 public:
  ValidateGoalNode(const std::string& name, const BT::NodeConfiguration& config,
                   NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

class TransformGoalNode : public BT::SyncActionNode {
 public:
  TransformGoalNode(const std::string& name, const BT::NodeConfiguration& config,
                    NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

class CheckRobotPoseNode : public BT::ConditionNode {
 public:
  CheckRobotPoseNode(const std::string& name,
                     const BT::NodeConfiguration& config,
                     NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

class ComputePCTPathNode : public BT::StatefulActionNode {
 public:
  ComputePCTPathNode(const std::string& name,
                     const BT::NodeConfiguration& config,
                     NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  void pctFeedback(
      const global_pct_planner::PlanPath3DFeedbackConstPtr& feedback);

  NavigationContext* context_;
  ros::Time action_started_;
  uint64_t pct_request_id_{0};
  unsigned int planning_attempt_{0};
  std::string planning_trigger_;
  std::string planning_reason_;
};

class ProcessReferencePathNode : public BT::SyncActionNode {
 public:
  ProcessReferencePathNode(const std::string& name,
                           const BT::NodeConfiguration& config,
                           NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

class ValidateReferencePathNode : public BT::ConditionNode {
 public:
  ValidateReferencePathNode(const std::string& name,
                            const BT::NodeConfiguration& config,
                            NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

class FollowReferencePathNode : public BT::StatefulActionNode {
 public:
  FollowReferencePathNode(const std::string& name,
                          const BT::NodeConfiguration& config,
                          NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  void scanFeedback(
      const navigation_msgs::FollowReferencePathFeedbackConstPtr& feedback);

  NavigationContext* context_;
  ros::Time action_started_;
  ros::WallTime action_started_wall_;
  mutable std::mutex feedback_mutex_;
  ros::WallTime last_feedback_wall_;
  bool received_feedback_{false};
  uint64_t request_id_{0};
  unsigned int planning_attempt_{0};
  std::string planning_trigger_;
  std::string planning_reason_;
};

class CheckGoalReachedNode : public BT::ConditionNode {
 public:
  CheckGoalReachedNode(const std::string& name,
                       const BT::NodeConfiguration& config,
                       NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

class MonitorProgressNode : public BT::SyncActionNode {
 public:
  MonitorProgressNode(const std::string& name,
                      const BT::NodeConfiguration& config,
                      NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

 private:
  NavigationContext* context_;
};

// Nav2-style two-child recovery controller. The first child is the main
// navigation task and the second child decides whether a failed task may be
// retried. A successful recovery restarts the first child from its root.
class NavigationRecoveryNode : public BT::ControlNode {
 public:
  NavigationRecoveryNode(const std::string& name,
                         const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts() { return {}; }
  void halt() override;

 private:
  BT::NodeStatus tick() override;
  std::size_t current_child_idx_{0};
};

// Selects exactly one typed recovery policy. SUCCESS means that this recovery
// permits one new execution of the main child; it is not a persistent request.
class RecoveryDispatcherNode : public BT::StatefulActionNode {
 public:
  RecoveryDispatcherNode(const std::string& name,
                         const BT::NodeConfiguration& config,
                         NavigationContext* context);
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

 private:
  bool waiting_for_environment_{false};
  bool waiting_for_dynamic_update_{false};
  RecoveryAction recovery_action_{RecoveryAction::TERMINAL};
  FailureDomain recovery_failure_domain_{FailureDomain::NONE};
  FailureCode recovery_failure_code_{FailureCode::NONE};
  ros::WallTime recovery_ready_at_;
  ros::WallTime recovery_deadline_;
  uint64_t dynamic_version_at_start_{0};
  NavigationContext* context_;
};

}  // namespace navigation_supervisor
