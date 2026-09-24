cmake_minimum_required(VERSION 2.8.3)
project(navigation_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  std_msgs)

add_action_files(FILES
  NavigateToPose.action
  FollowReferencePath.action)

add_message_files(FILES
  NavigationState.msg)

generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs)

catkin_package(CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  nav_msgs
  std_msgs)
