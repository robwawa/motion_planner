cmake_minimum_required(VERSION 2.8.3)
project(dynamic_perception_3d)

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
  geometry_msgs
  pcl_conversions
  pcl_ros
  motion_planner_log
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search segmentation)

######################
### Catkin 配置
######################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_perception_3d_core
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
                 tf2 tf2_eigen tf2_ros visualization_msgs motion_planner_log
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Core library contains map filtering, clustering and obstacle lifecycle code.
add_library(dynamic_perception_3d_core
  src/static_map_filter.cpp
  src/dynamic_clusterer.cpp
  src/visibility_clearer.cpp
  src/obstacle_manager.cpp
)
target_link_libraries(dynamic_perception_3d_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ROS executable owns node initialization and publishes the dynamic snapshot.
add_executable(dynamic_perception_3d_node
  src/dynamic_perception_node.cpp
  src/dynamic_perception_main.cpp
)
target_link_libraries(dynamic_perception_3d_node
  dynamic_perception_3d_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  ######################
  ### 测试
  ######################
  find_package(rostest REQUIRED)
  catkin_add_gtest(dynamic_perception_3d_core_test test/core_test.cpp)
  if(TARGET dynamic_perception_3d_core_test)
    target_link_libraries(dynamic_perception_3d_core_test
      gtest_main dynamic_perception_3d_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
    target_compile_definitions(dynamic_perception_3d_core_test PRIVATE
      DYNAMIC_PERCEPTION_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endif()
  add_rostest(test/node_integration.test)
endif()

#############
## 安装规则 ##
#############
install(TARGETS dynamic_perception_3d_core dynamic_perception_3d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
