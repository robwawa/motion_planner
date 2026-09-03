cmake_minimum_required(VERSION 3.0.2)
project(dynamic_perception_3d)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_eigen
  tf2_ros
  visualization_msgs
)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search segmentation)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_perception_3d_core
  CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
                 tf2 tf2_eigen tf2_ros visualization_msgs
  DEPENDS PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(dynamic_perception_3d_core
  src/static_map_filter.cpp
  src/dynamic_clusterer.cpp
  src/visibility_clearer.cpp
  src/obstacle_manager.cpp
)
target_link_libraries(dynamic_perception_3d_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(dynamic_perception_3d_node
  src/dynamic_perception_node.cpp
  src/dynamic_perception_main.cpp
)
target_link_libraries(dynamic_perception_3d_node
  dynamic_perception_3d_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
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

install(TARGETS dynamic_perception_3d_core dynamic_perception_3d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
