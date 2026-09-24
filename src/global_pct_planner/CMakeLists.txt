cmake_minimum_required(VERSION 2.8.3)
project(global_pct_planner)

######################
### 编译配置
######################
# 与移动硬盘版 gdlp3dSlam 保持一致：Release、C++17、优化和线程支持。
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -Wall -pthread")

######################
### ROS 与系统依赖
######################
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  sensor_msgs
  std_msgs
  motion_planner_log
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED)

######################
### Action、Message 生成
######################
add_action_files(FILES PlanPath3D.action)
add_message_files(FILES PctTerrainMap.msg)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs nav_msgs std_msgs)

######################
### Catkin 导出
######################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_pct_core
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_runtime nav_msgs
    pcl_conversions pcl_ros roscpp roslib sensor_msgs std_msgs
    motion_planner_log
  DEPENDS PCL
)

######################
### PCT native 依赖路径
######################
# native 是本包实际需要的 PCT 算法源码；GTSAM、METIS、OSQP 仍从已有
# PCT 第三方安装目录查找，不复制源码、不把第三方目录纳入本包安装结果。
get_filename_component(PCT_NATIVE_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/native REALPATH)
set(PCT_DEPENDENCY_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/../pct_planner/planner/lib/3rdparty
  CACHE PATH "Root directory containing the local GTSAM and OSQP installs")
set(PCT_GTSAM_DIR ${PCT_DEPENDENCY_ROOT}/gtsam-4.1.1/install/lib/cmake/GTSAM
    CACHE PATH "GTSAM config directory used by the existing PCT native core")
get_filename_component(PCT_GTSAM_LIB_DIR ${PCT_GTSAM_DIR}/../.. REALPATH)
set(PCT_METIS_LIB_DIR
    ${PCT_DEPENDENCY_ROOT}/gtsam-4.1.1/build/gtsam/3rdparty/metis/libmetis
    CACHE PATH "Directory containing the vendored GTSAM METIS shared library")
set(PCT_METIS_LIBRARY ${PCT_METIS_LIB_DIR}/libmetis-gtsam.so)
set(PCT_OSQP_DIR ${PCT_DEPENDENCY_ROOT}/osqp/install
    CACHE PATH "OSQP install prefix used by the native GPMP core")
if(NOT EXISTS ${PCT_GTSAM_DIR})
  message(FATAL_ERROR
    "GTSAM was not found at ${PCT_GTSAM_DIR}. Build the existing PCT third-party dependency first.")
endif()
if(NOT EXISTS ${PCT_METIS_LIBRARY})
  message(FATAL_ERROR
    "GTSAM METIS was not found at ${PCT_METIS_LIBRARY}. Build the existing PCT third-party dependency first.")
endif()

# 只导入 GPMP 运行时需要的 GTSAM/OSQP 组件。不要 add_subdirectory 旧的
# planner/lib 顶层目录，因为它还会生成不属于本节点运行时的 Python/pybind
# 模块。PCT_* 路径均可由构建参数覆盖，避免写死机器路径。
set(GTSAM_DIR ${PCT_GTSAM_DIR} CACHE PATH
    "GTSAM config directory used by the existing PCT native core" FORCE)
find_package(GTSAM REQUIRED)
set(osqp_DIR ${PCT_OSQP_DIR}/lib/cmake/osqp CACHE PATH
    "OSQP CMake package directory used by the native GPMP core" FORCE)
find_package(osqp REQUIRED)

set(PCT_NATIVE_INCLUDE_DIR ${PCT_NATIVE_ROOT}/src)
set(PCT_OSQP_INCLUDE_DIR ${PCT_OSQP_DIR}/include)
set(PCT_NATIVE_COMPILE_OPTIONS "-Wall -Wextra -O2 -march=native")

######################
### PCT native 算法库
######################
# native 库拆开构建，便于 global_pct_core 复用 A*、地图、平滑和 GPMP
# 组件，也能让 catkin install 正确安装每个运行时动态库。
function(configure_pct_native_target target)
  target_include_directories(${target} PUBLIC
    ${PCT_NATIVE_INCLUDE_DIR}
    ${PCT_OSQP_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIRS})
  set_target_properties(${target} PROPERTIES COMPILE_FLAGS
    "${PCT_NATIVE_COMPILE_OPTIONS}")
endfunction()

add_library(a_star_search SHARED
  ${PCT_NATIVE_ROOT}/src/a_star/a_star_search.cc)
configure_pct_native_target(a_star_search)

add_library(map_manager SHARED
  ${PCT_NATIVE_ROOT}/src/map_manager/dense_elevation_map.cc)
configure_pct_native_target(map_manager)

set(PCT_SMOOTHING_ROOT ${PCT_NATIVE_ROOT}/src/common/smoothing)
add_library(common_smoothing SHARED
  ${PCT_SMOOTHING_ROOT}/affine_constraint.cc
  ${PCT_SMOOTHING_ROOT}/discrete_points_math.cc
  ${PCT_SMOOTHING_ROOT}/osqp_spline1d_solver.cc
  ${PCT_SMOOTHING_ROOT}/osqp_spline2d_solver.cc
  ${PCT_SMOOTHING_ROOT}/polynomialxd.cc
  ${PCT_SMOOTHING_ROOT}/spline1d_constraint.cc
  ${PCT_SMOOTHING_ROOT}/spline1d_kernel_helper.cc
  ${PCT_SMOOTHING_ROOT}/spline1d_kernel.cc
  ${PCT_SMOOTHING_ROOT}/spline1d_seg.cc
  ${PCT_SMOOTHING_ROOT}/spline1d.cc
  ${PCT_SMOOTHING_ROOT}/spline2d_constraint.cc
  ${PCT_SMOOTHING_ROOT}/spline2d_kernel.cc
  ${PCT_SMOOTHING_ROOT}/spline2d_seg.cc
  ${PCT_SMOOTHING_ROOT}/spline2d.cc
  ${PCT_SMOOTHING_ROOT}/solver/osqp/osqp_interface.cc
  ${PCT_SMOOTHING_ROOT}/utils/math.cc)
configure_pct_native_target(common_smoothing)
target_link_libraries(common_smoothing PUBLIC osqp::osqpstatic)

set(PCT_GPMP_ROOT ${PCT_NATIVE_ROOT}/src/trajectory_optimization)
add_library(gpmp_optimizer SHARED
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors/gp_prior_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors/gp_obstacle_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors/gp_interpolate_obstacle_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors/gp_heading_rate_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors_wnoa/gp_prior_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors_wnoa/gp_obstacle_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors_wnoa/gp_interpolate_obstacle_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/factors_origin/gp_prior_factor.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/interpolator/wnoa_trajectory_interpolator.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/gpmp_optimizer.cc
  ${PCT_GPMP_ROOT}/gpmp_optimizer/gpmp_optimizer_wnoa.cc
  ${PCT_GPMP_ROOT}/height_smoother/height_smoother.cc)
configure_pct_native_target(gpmp_optimizer)
target_link_libraries(gpmp_optimizer PUBLIC gtsam map_manager common_smoothing)

add_library(ele_planner_lib SHARED
  ${PCT_NATIVE_ROOT}/src/ele_planner/offline_ele_planner.cc)
configure_pct_native_target(ele_planner_lib)
target_link_libraries(ele_planner_lib PUBLIC
  a_star_search gpmp_optimizer map_manager)

######################
### global PCT 核心库
######################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PCT_NATIVE_ROOT}/src
)

add_library(global_pct_core
  src/tomogram.cpp
  src/tomogram_io.cpp
  src/dynamic_obstacle_layer.cpp
  src/planner_core.cpp
)
set_target_properties(global_pct_core PROPERTIES COMPILE_FLAGS
  "-Wall -Wextra -O2 ${OpenMP_CXX_FLAGS}")
target_link_libraries(global_pct_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ele_planner_lib
  gpmp_optimizer
  map_manager
  common_smoothing
  ${PCT_METIS_LIBRARY}
)
add_dependencies(global_pct_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

######################
### tomography 与规划节点
######################
add_executable(pct_tomography_node src/pct_tomography_node.cpp)
set_target_properties(pct_tomography_node PROPERTIES COMPILE_FLAGS
  "-Wall -Wextra -O2 ${OpenMP_CXX_FLAGS}")
target_link_libraries(pct_tomography_node global_pct_core ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS})
add_dependencies(pct_tomography_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(global_pct_planner_node src/global_pct_planner_node.cpp)
set_target_properties(global_pct_planner_node PROPERTIES COMPILE_FLAGS
  "-Wall -Wextra -O2 ${OpenMP_CXX_FLAGS}")
target_link_libraries(global_pct_planner_node global_pct_core ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS})
add_dependencies(global_pct_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

######################
### 安装规则
######################
# 安装节点、核心库和 native 运行库；资源按包相对路径安装，确保 source
# install/setup.bash 后 launch/config/地图可直接发现。
install(TARGETS global_pct_core pct_tomography_node global_pct_planner_node
  ele_planner_lib gpmp_optimizer map_manager common_smoothing a_star_search
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch config rsc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS
  tools/convert_pickle_to_pctm.py
  tools/compare_legacy_pct.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(global_pct_tomogram_test test/tomogram_test.cpp)
  if(TARGET global_pct_tomogram_test)
    target_link_libraries(global_pct_tomogram_test global_pct_core ${catkin_LIBRARIES})
    add_dependencies(global_pct_tomogram_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
  catkin_add_gtest(global_pct_dynamic_obstacle_test test/dynamic_obstacle_layer_test.cpp)
  if(TARGET global_pct_dynamic_obstacle_test)
    target_link_libraries(global_pct_dynamic_obstacle_test global_pct_core ${catkin_LIBRARIES})
    add_dependencies(global_pct_dynamic_obstacle_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/dense_elevation_map_dynamic_test.cpp")
    catkin_add_gtest(global_pct_dense_elevation_map_dynamic_test
      test/dense_elevation_map_dynamic_test.cpp)
    if(TARGET global_pct_dense_elevation_map_dynamic_test)
      target_link_libraries(global_pct_dense_elevation_map_dynamic_test map_manager)
      set_target_properties(global_pct_dense_elevation_map_dynamic_test PROPERTIES
        COMPILE_FLAGS "${PCT_NATIVE_COMPILE_OPTIONS}")
      add_dependencies(global_pct_dense_elevation_map_dynamic_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    endif()
  else()
    message(STATUS "Skipping dense elevation map test: source file is missing")
  endif()
  catkin_add_gtest(global_pct_action_interface_test test/action_interface_test.cpp)
  if(TARGET global_pct_action_interface_test)
    target_link_libraries(global_pct_action_interface_test ${catkin_LIBRARIES})
    add_dependencies(global_pct_action_interface_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
  catkin_add_gtest(global_pct_planner_core_test test/planner_core_test.cpp)
  if(TARGET global_pct_planner_core_test)
    target_link_libraries(global_pct_planner_core_test global_pct_core ${catkin_LIBRARIES})
    add_dependencies(global_pct_planner_core_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
  catkin_add_gtest(global_pct_pctm_fixture_test test/pctm_fixture_test.cpp)
  if(TARGET global_pct_pctm_fixture_test)
    target_link_libraries(global_pct_pctm_fixture_test global_pct_core ${catkin_LIBRARIES})
    target_compile_definitions(global_pct_pctm_fixture_test PRIVATE
      GLOBAL_PCT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    add_dependencies(global_pct_pctm_fixture_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
  catkin_add_gtest(global_pct_legacy_astar_integration_test
    test/legacy_astar_integration_test.cpp)
  if(TARGET global_pct_legacy_astar_integration_test)
    target_link_libraries(global_pct_legacy_astar_integration_test
      global_pct_core a_star_search ${catkin_LIBRARIES})
    add_dependencies(global_pct_legacy_astar_integration_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  endif()
endif()

# GTSAM 的私有 METIS 不一定在系统 linker cache 中。把可配置的本地路径
# 写入 build/install RPATH，使测试和 install 后启动不依赖手工设置
# LD_LIBRARY_PATH；这里不复制第三方源码或构建目录。
set(GLOBAL_PCT_RPATH "${PCT_GTSAM_LIB_DIR};${PCT_METIS_LIB_DIR}")
foreach(global_pct_target global_pct_core pct_tomography_node global_pct_planner_node
        ele_planner_lib gpmp_optimizer map_manager common_smoothing a_star_search
        global_pct_tomogram_test global_pct_dynamic_obstacle_test
        global_pct_dense_elevation_map_dynamic_test
        global_pct_action_interface_test global_pct_planner_core_test
        global_pct_pctm_fixture_test global_pct_legacy_astar_integration_test)
  if(TARGET ${global_pct_target})
    set_target_properties(${global_pct_target} PROPERTIES
      BUILD_RPATH "${GLOBAL_PCT_RPATH}"
      INSTALL_RPATH "${GLOBAL_PCT_RPATH}")
    # catkin_add_gtest may append its own link settings after BUILD_RPATH is
    # evaluated.  Keep METIS as a direct runtime search path for every test.
    set_property(TARGET ${global_pct_target} APPEND_STRING PROPERTY LINK_FLAGS
      " -Wl,-rpath,${PCT_METIS_LIB_DIR} -Wl,--disable-new-dtags")
  endif()
endforeach()

if(CATKIN_ENABLE_TESTING)
  # libgtsam is a prebuilt local dependency whose own DT_RUNPATH does not
  # propagate to its METIS dependency.  ctest inherits this path explicitly,
  # so `catkin_make run_tests` works without a shell-specific environment.
  set(GLOBAL_PCT_TEST_LD_LIBRARY_PATH
      "${PCT_METIS_LIB_DIR}:${PCT_GTSAM_LIB_DIR}:$ENV{LD_LIBRARY_PATH}")
  foreach(global_pct_test global_pct_tomogram_test
          global_pct_dynamic_obstacle_test
          global_pct_dense_elevation_map_dynamic_test
          global_pct_action_interface_test global_pct_planner_core_test
          global_pct_pctm_fixture_test global_pct_legacy_astar_integration_test)
    if(TARGET ${global_pct_test})
      set_tests_properties(_ctest_${PROJECT_NAME}_gtest_${global_pct_test} PROPERTIES
        ENVIRONMENT "LD_LIBRARY_PATH=${GLOBAL_PCT_TEST_LD_LIBRARY_PATH}")
    endif()
  endforeach()
endif()
