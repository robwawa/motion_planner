# Install script for directory: /home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/wa/inspection/3D_motion_planner/motion_planner/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/wa/inspection/3D_motion_planner/motion_planner/build/pct_planner/catkin_generated/installspace/pct_planner.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/cmake" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/build/pct_planner/catkin_generated/installspace/pct_plannerConfig.cmake"
    "/home/wa/inspection/3D_motion_planner/motion_planner/build/pct_planner/catkin_generated/installspace/pct_plannerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner" TYPE FILE FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pct_planner" TYPE PROGRAM FILES "/home/wa/inspection/3D_motion_planner/motion_planner/build/pct_planner/catkin_generated/installspace/pct_tomography_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pct_planner" TYPE PROGRAM FILES "/home/wa/inspection/3D_motion_planner/motion_planner/build/pct_planner/catkin_generated/installspace/pct_planner_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner" TYPE DIRECTORY FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/launch"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/rsc"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/tomography" TYPE DIRECTORY FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/tomography/config" REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE REGEX "/[^/]*\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner" TYPE DIRECTORY FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/config" REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE REGEX "/[^/]*\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/tomography" TYPE DIRECTORY FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/tomography/scripts" FILES_MATCHING REGEX "/[^/]*\\.py$" REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner" TYPE DIRECTORY FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/scripts" FILES_MATCHING REGEX "/[^/]*\\.py$" REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/a_star.cpython-38-x86_64-linux-gnu.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/ele_planner.cpython-38-x86_64-linux-gnu.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/libcommon_smoothing.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/py_map_manager.cpython-38-x86_64-linux-gnu.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/traj_opt.cpython-38-x86_64-linux-gnu.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4.1.1"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam_unstable.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam_unstable.so.4"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam_unstable.so.4.1.1"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libmetis-gtsam.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" TYPE FILE RENAME "libgtsam.so.4" FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4.1.1")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib" TYPE FILE RENAME "libgtsam.so" FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/libgtsam.so.4.1.1")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/build/src/common/smoothing" TYPE FILE FILES "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/common/smoothing/libcommon_smoothing.so")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/build/src/a_star" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/a_star/a_star.cpython-38-x86_64-linux-gnu.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/a_star/liba_star_search.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/build/src/map_manager" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/map_manager/libmap_manager.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/map_manager/py_map_manager.cpython-38-x86_64-linux-gnu.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/build/src/trajectory_optimization" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/trajectory_optimization/libgpmp_optimizer.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/trajectory_optimization/traj_opt.cpython-38-x86_64-linux-gnu.so"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/pct_planner/planner/lib/build/src/ele_planner" TYPE FILE FILES
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/ele_planner/ele_planner.cpython-38-x86_64-linux-gnu.so"
    "/home/wa/inspection/3D_motion_planner/motion_planner/src/pct_planner/planner/lib/build/src/ele_planner/libele_planner_lib.so"
    )
endif()

