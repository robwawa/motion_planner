cmake_minimum_required(VERSION 3.0.2)
project(pct_planner)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

add_action_files(
  FILES
  PlanPath3D.action
)

add_message_files(
  FILES
  PctTerrainMap.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime nav_msgs rospy sensor_msgs std_msgs)

catkin_install_python(PROGRAMS
  scripts/pct_tomography_node.py
  scripts/pct_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rsc config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY tomography/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tomography
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY planner/config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Keep the Python implementation available in an installed package.  The
# native planner libraries are produced by planner/build*.sh and are copied
# when they exist; source-space launches continue to use the build outputs.
install(DIRECTORY tomography/scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tomography
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY planner/scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install only runtime libraries, not the vendored source trees and CMake
# build directories.  Keep the relative layout expected by the Python
# wrapper so both devel-space and install-space launches use the same paths.
file(GLOB PCT_ROOT_NATIVE_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/*.so*"
)
file(GLOB PCT_GTSAM_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/3rdparty/gtsam-4.1.1/install/lib/*.so*"
)
file(GLOB PCT_SMOOTHING_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/build/src/common/smoothing/*.so*"
)
file(GLOB PCT_ASTAR_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/build/src/a_star/*.so*"
)
file(GLOB PCT_MAP_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/build/src/map_manager/*.so*"
)
file(GLOB PCT_TRAJ_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/build/src/trajectory_optimization/*.so*"
)
file(GLOB PCT_ELE_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/build/src/ele_planner/*.so*"
)

if(PCT_ROOT_NATIVE_LIBS)
  install(FILES ${PCT_ROOT_NATIVE_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib)
endif()
if(PCT_GTSAM_LIBS)
  install(FILES ${PCT_GTSAM_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/3rdparty/gtsam-4.1.1/install/lib)
  # CMake may dereference symlinks when installing FILES.  Publish the
  # SONAME and linker names explicitly so the installed package can resolve
  # libgtsam.so.4 and libgtsam.so without relying on the source tree.
  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4.1.1"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/3rdparty/gtsam-4.1.1/install/lib
    RENAME libgtsam.so.4)
  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4.1.1"
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/3rdparty/gtsam-4.1.1/install/lib
    RENAME libgtsam.so)
endif()
if(PCT_SMOOTHING_LIBS)
  install(FILES ${PCT_SMOOTHING_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/build/src/common/smoothing)
endif()
if(PCT_ASTAR_LIBS)
  install(FILES ${PCT_ASTAR_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/build/src/a_star)
endif()
if(PCT_MAP_LIBS)
  install(FILES ${PCT_MAP_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/build/src/map_manager)
endif()
if(PCT_TRAJ_LIBS)
  install(FILES ${PCT_TRAJ_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/build/src/trajectory_optimization)
endif()
if(PCT_ELE_LIBS)
  install(FILES ${PCT_ELE_LIBS}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/planner/lib/build/src/ele_planner)
endif()
