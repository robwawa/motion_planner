#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <atomic>
#include <actionlib/server/simple_action_server.h>
#include <boost/bind/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <navigation_msgs/FollowReferencePathAction.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mutex>
#include <memory>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner/Bspline.h>
#include <scan_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/reference_path_z.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      BRAKE_FOR_NEW_TARGET,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner::DataDisp data_disp_;

    /* parameters */
    int navi_mode_; // 1 manual select, 2 hard code, 3 reference path
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::Vector3d> preset_waypoints_;
    int waypoint_num_;
    double planning_horizon_;
    double emergency_time_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    double reference_path_min_distance_;
    double reference_path_simplify_tolerance_;
    std::string reference_path_topic_;
    std::string reference_path_z_mode_;
    std::string reference_path_mode_;
    bool adaptive_horizon_enabled_;
    double adaptive_horizon_min_;
    double adaptive_horizon_max_;
    double adaptive_horizon_curvature_gain_;
    double adaptive_horizon_slope_gain_;
    double adaptive_horizon_slope_smoothing_window_;
    bool direction_change_brake_enabled_;
    double direction_change_threshold_deg_;
    double direction_change_min_speed_;
    double direction_change_stop_speed_;
    double direction_change_brake_acc_ratio_;
    double progress_timeout_sec_{8.0};
    double progress_min_distance_{0.10};
    double progress_tracking_error_tolerance_{1.5};
    double goal_reached_tolerance_{0.5};
    double endpoint_truncation_tolerance_{0.05};
    int max_progress_replan_attempts_{2};
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    bool emergency_stop_result_pending_{false};
    std::string emergency_stop_reason_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int progress_replan_attempts_{0};
    bool progress_watchdog_initialized_{false};
    bool tracking_error_initialized_{false};
    bool active_route_endpoint_truncated_{false};
    Eigen::Vector3d progress_anchor_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d requested_route_goal_{Eigen::Vector3d::Zero()};
    ros::WallTime progress_anchor_time_;
    ros::WallTime tracking_error_since_;
    double progress_anchor_remaining_distance_{0.0};
    int max_replan_fail_count_{1000};
    ros::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    double final_yaw_{0.0};
    bool have_final_yaw_{false};
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    Eigen::Vector2d pending_target_direction_{Eigen::Vector2d::Zero()};
    std::vector<Eigen::Vector3d> active_waypoints_;
    ReferencePathZProfile reference_path_z_profile_;
    double reference_path_z_progress_{0.0};
    int current_wp_;

    bool flag_escape_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber goal_sub_, odom_sub_, path_sub_, go2_execution_frozen_sub_;
    ros::Publisher new_pub_, bspline_pub_, data_disp_pub_, self_inflation_pub_,
        global_reference_path_pub_, downsampled_reference_path_pub_, reference_path_z_profile_pub_;

    using FollowActionServer =
        actionlib::SimpleActionServer<navigation_msgs::FollowReferencePathAction>;
    FollowActionServer follow_action_server_;
    std::mutex follow_mutex_;
    navigation_msgs::FollowReferencePathGoalConstPtr pending_follow_goal_;
    bool follow_request_pending_{false};
    bool follow_result_ready_{false};
    bool follow_cancel_requested_{false};
    uint64_t cancel_request_mission_id_{0};
    uint64_t cancel_request_route_id_{0};
    uint8_t follow_result_code_{navigation_msgs::FollowReferencePathResult::FAILED};
    std::string follow_result_message_;
    std::string follow_result_reason_code_;
    std::string follow_state_;
    double follow_distance_remaining_{0.0};
    geometry_msgs::PoseStamped follow_execution_goal_;
    geometry_msgs::PoseStamped follow_final_pose_;
    double follow_result_remaining_distance_{0.0};
    bool follow_result_endpoint_truncated_{false};
    uint64_t active_mission_id_{0};
    uint64_t active_route_id_{0};
    uint64_t last_goal_mission_id_{0};
    uint64_t last_goal_route_id_{0};
    bool have_last_goal_id_{false};

    // Route reception and route activation are deliberately separate. The
    // action callback queues a command, while the FSM timer owns planner data
    // and activates the newest route when it is safe to do so.
    nav_msgs::Path pending_route_;
    uint64_t pending_route_mission_id_{0};
    uint64_t pending_route_id_{0};
    uint64_t pending_route_request_id_{0};
    uint32_t pending_route_planning_attempt_{0};
    std::string pending_route_trigger_;
    std::string pending_route_trigger_reason_;
    uint64_t pending_route_generation_{0};
    bool pending_route_ready_{false};
    uint64_t pending_follow_generation_{0};
    std::atomic<uint64_t> route_generation_{0};
    uint64_t active_execution_mission_id_{0};
    uint64_t active_execution_route_id_{0};
    uint64_t active_execution_request_id_{0};
    uint32_t active_execution_planning_attempt_{0};
    std::string active_execution_trigger_;
    std::string active_execution_trigger_reason_;
    uint64_t active_execution_generation_{0};

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    bool shouldBrakeForDirectionChange();
    bool startDirectionChangeBrake();
    void updatePendingTargetDirection(const Eigen::Vector3d &start,
                                      const std::vector<Eigen::Vector3d> &points);
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planGlobalTrajbyGivenWps();
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool planNextWaypoint();
    bool isWaypointSequenceMode() const;
    bool usePolylineRollingWindow() const;
    bool activatePendingRoute(std::string &error);
    void scheduleRouteExecution();
    void setFollowState(const std::string &state);
    double projectReferencePathProgress(const Eigen::Vector3d &point,
                                        double min_progress,
                                        double max_progress) const;
    bool adjustGlobalTargetIfOccupied();
    void getLocalTarget();
    void finishProcess();
    void updateProgressWatchdog();
    void publishSelfInflationMarker();
    void publishGlobalReferencePath();
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void rvizGoalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void waypointCallback(const nav_msgs::PathConstPtr &msg);
    void pathCallback(const nav_msgs::PathConstPtr &msg);
    bool acceptReferencePath(const nav_msgs::Path &msg, std::string &error);
    void followReferencePathExecute(
        const navigation_msgs::FollowReferencePathGoalConstPtr &goal);
    void processPendingFollowRequest();
    void finishFollowAction(uint8_t result, const std::string &message,
                            uint64_t expected_mission_id = 0,
                            uint64_t expected_route_id = 0,
                            uint64_t expected_route_generation = 0,
                            const std::string &reason_code = "");
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::BoolConstPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
        : follow_action_server_(node_, "/scan/follow_reference_path",
                                boost::bind(&SCANReplanFSM::followReferencePathExecute,
                                            this, boost::placeholders::_1),
                                false)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
