#include <ros/ros.h>
#include <motion_planner_log/logging.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/scan_replan_fsm.h>

using namespace scan_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_planner_node");
  motion_planner_log::initialize("scan_planner_node", argv[0]);
  MOTION_PLANNER_LOG_INFO("Node starting: scan planner.");
  ros::NodeHandle nh("~");

  SCANReplanFSM scan_replan;

  scan_replan.init(nh);
  MOTION_PLANNER_LOG_INFO("Ready: scan replanning FSM initialized.");

  ros::Duration(1.0).sleep();
  ros::spin();

  MOTION_PLANNER_LOG_INFO("Node shutting down: scan planner.");

  return 0;
}
