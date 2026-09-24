cmake_minimum_required(VERSION 2.8.3)
project(navigation_supervisor)

######################
### 编译配置
######################
# 与工程其他 ROS 包保持一致，统一使用 Release、C++17 和线程支持。
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall -pthread")

######################
### ROS 依赖
######################
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  global_pct_planner
  motion_planner_log
  nav_msgs
  navigation_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

######################
### BehaviorTree.CPP 依赖
######################
# 行为树库负责导航流程编排；其头文件和动态库由系统/ROS 环境提供，
# 本包只链接运行时所需的库，不复制第三方源码。
find_package(behaviortree_cpp_v3 REQUIRED)
find_path(BEHAVIORTREE_CPP_V3_INCLUDE_DIR
  behaviortree_cpp_v3/bt_factory.h
  PATHS ${behaviortree_cpp_v3_INCLUDE_DIRS})
get_filename_component(BEHAVIORTREE_CPP_V3_PREFIX
  ${BEHAVIORTREE_CPP_V3_INCLUDE_DIR} DIRECTORY)
find_library(BEHAVIORTREE_CPP_V3_LIBRARY
  NAMES behaviortree_cpp_v3
  PATHS ${behaviortree_cpp_v3_LIBRARY_DIRS}
        ${BEHAVIORTREE_CPP_V3_PREFIX}/lib)
if(NOT BEHAVIORTREE_CPP_V3_INCLUDE_DIR OR NOT BEHAVIORTREE_CPP_V3_LIBRARY)
  message(FATAL_ERROR "BehaviorTree.CPP v3 headers/library were not found")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib geometry_msgs global_pct_planner
    motion_planner_log nav_msgs navigation_msgs roscpp tf2
    tf2_geometry_msgs tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS}
  ${BEHAVIORTREE_CPP_V3_INCLUDE_DIR})

######################
### 导航监督节点
######################
# 保持现有目标名称，避免影响 launch 和外部调用。
add_executable(navigation_supervisor_node
  src/navigation_supervisor_node.cpp
  src/navigation_context.cpp
  src/bt_nodes.cpp
  src/reference_path_processor.cpp
)
set_target_properties(navigation_supervisor_node PROPERTIES COMPILE_FLAGS
  "-Wall -Wextra -O2 -pthread"
)
target_link_libraries(navigation_supervisor_node
  ${catkin_LIBRARIES}
  ${BEHAVIORTREE_CPP_V3_LIBRARY})
add_dependencies(navigation_supervisor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

######################
### 安装规则
######################
install(TARGETS navigation_supervisor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

######################
### 测试
######################
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(reference_path_processor_test
    test/reference_path_processor_test.cpp
    src/reference_path_processor.cpp
  )
  if(TARGET reference_path_processor_test)
    target_link_libraries(reference_path_processor_test
      ${catkin_LIBRARIES}
      gtest_main
    )
    add_dependencies(reference_path_processor_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
  endif()

  catkin_add_gtest(recovery_policy_test
    test/recovery_policy_test.cpp
    src/navigation_context.cpp
    src/reference_path_processor.cpp
  )
  if(TARGET recovery_policy_test)
    target_link_libraries(recovery_policy_test ${catkin_LIBRARIES} gtest_main)
    add_dependencies(recovery_policy_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
  endif()
endif()
