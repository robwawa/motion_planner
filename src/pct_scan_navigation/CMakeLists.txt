cmake_minimum_required(VERSION 3.0.2)
project(pct_scan_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  interactive_markers
  nav_msgs
  pct_planner
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/navigation_manager.py
  scripts/goal_interactive_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
