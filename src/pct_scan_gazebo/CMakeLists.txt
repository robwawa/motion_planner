cmake_minimum_required(VERSION 2.8.3)
project(pct_scan_gazebo)

######################
### 编译配置
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -pthread")

######################
### 依赖包
######################
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  gazebo_msgs
  gazebo_ros
  global_pct_planner
  geometry_msgs
  hardware_interface
  message_generation
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  motion_planner_log
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  urdf
)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

######################
### 消息生成
######################
add_message_files(FILES MotorCmd.msg MotorState.msg)
generate_messages(DEPENDENCIES std_msgs)

######################
### Catkin 配置
######################
catkin_package(
  INCLUDE_DIRS include control_include
  LIBRARIES pct_scan_gazebo_livox pct_scan_gazebo_control pct_scan_gazebo_contact pct_scan_gazebo_draw_force
  CATKIN_DEPENDS controller_interface gazebo_msgs gazebo_ros global_pct_planner geometry_msgs hardware_interface message_runtime motion_planner_log nav_msgs pluginlib realtime_tools roscpp rospy sensor_msgs std_msgs tf tf2_ros urdf
)

include_directories(
  include control_include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

######################
### Gazebo 插件和节点
######################
add_library(pct_scan_gazebo_livox SHARED
  livox_src/livox_points_plugin.cpp
  livox_src/livox_ode_multiray_shape.cpp
)
target_link_libraries(pct_scan_gazebo_livox ${catkin_LIBRARIES} RayPlugin protobuf)

add_library(pct_scan_gazebo_control SHARED
  control_src/joint_controller.cpp
  control_src/unitree_joint_control_tool.cpp
)
add_dependencies(pct_scan_gazebo_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pct_scan_gazebo_control ${catkin_LIBRARIES})

add_library(pct_scan_gazebo_contact SHARED plugin/foot_contact_plugin.cc)
target_link_libraries(pct_scan_gazebo_contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(pct_scan_gazebo_draw_force SHARED plugin/draw_force_plugin.cc)
target_link_libraries(pct_scan_gazebo_draw_force ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(state_from_gazebo state_from_gazebo.cpp)
target_link_libraries(state_from_gazebo ${catkin_LIBRARIES})

#############
## 安装规则 ##
#############
install(TARGETS
  pct_scan_gazebo_livox
  pct_scan_gazebo_control
  pct_scan_gazebo_contact
  pct_scan_gazebo_draw_force
  state_from_gazebo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS pointcloud2livox.py rl_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY a1_description/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/a1_description
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)
install(FILES unitree_controller_plugins.xml policy_act_inference_stair.pt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
