cmake_minimum_required(VERSION 2.8.3)
project(pct_scan_navigation)

######################
### 依赖包和 Catkin 配置
######################
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  interactive_markers
  nav_msgs
  global_pct_planner
  motion_planner_log
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(CATKIN_DEPENDS motion_planner_log global_pct_planner)

if(CATKIN_ENABLE_TESTING)
  ######################
  ### 测试
  ######################
  catkin_add_nosetests(test)
endif()

#############
## 安装规则 ##
#############
catkin_install_python(PROGRAMS
  scripts/navigation_manager.py
  scripts/goal_interactive_marker.py
  scripts/replan_cycle_budget.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
