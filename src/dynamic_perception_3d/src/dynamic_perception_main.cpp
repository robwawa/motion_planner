#include <exception>

#include <ros/ros.h>

#include "dynamic_perception_3d/dynamic_perception_node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_perception_3d");
  try {
    dynamic_perception_3d::DynamicPerceptionNode node(ros::NodeHandle(),
                                                       ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("dynamic_perception_3d failed to start: %s", error.what());
    return 1;
  }
  return 0;
}
