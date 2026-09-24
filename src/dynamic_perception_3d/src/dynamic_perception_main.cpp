#include <exception>
#include <motion_planner_log/logging.h>

#include <ros/ros.h>

#include "dynamic_perception_3d/dynamic_perception_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_perception_3d");
  motion_planner_log::initialize("dynamic_perception_3d", "dynamic_perception_3d", argv[0]);
  MOTION_PLANNER_LOG_INFO("Node starting: dynamic perception.");
  try {
    dynamic_perception_3d::DynamicPerceptionNode node(ros::NodeHandle(),
                                                       ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    MOTION_PLANNER_LOG_ERROR("dynamic_perception_3d failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
