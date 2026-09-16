cmake_minimum_required(VERSION 2.8.3)
project(scan_planner)

# 统一编译选项：主规划器及其内部工具目标使用同一套 C++ 标准和优化选项。
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O2 -Wall -pthread")
option(USE_GPU "Build the optional OpenGL point-cloud renderer" OFF)

# 系统依赖。
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization)
find_package(OpenMP REQUIRED)

# 小型子模块已取消 ROS 包边界，以下依赖由 scan_planner 统一声明。
find_package(catkin REQUIRED COMPONENTS
  roscpp rospy roslib
  std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf
  message_filters cv_bridge pcl_ros pcl_conversions
  message_generation motion_planner_log pct_planner)

# GPU 依赖仅在启用 GPU 渲染器时检查。
if(USE_GPU)
  find_package(GLEW REQUIRED)
  find_package(OpenGL REQUIRED)
  find_package(glfw3 QUIET)
  if(NOT glfw3_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/local_sensing/third_party/glfw/usr")
    set(glfw3_FOUND TRUE)
    set(glfw3_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/local_sensing/third_party/glfw/usr/include")
    set(glfw3_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/local_sensing/third_party/glfw/usr/lib/x86_64-linux-gnu/libglfw.so.3.3")
  endif()
  if(NOT glfw3_FOUND)
    message(FATAL_ERROR "USE_GPU=ON requires glfw3")
  endif()
endif()

# 消息生成。
add_message_files(FILES Bspline.msg DataDisp.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# 对外导出统一包头文件和库目标。
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plan_env path_searching bspline_opt traj_utils pose_utils
  CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs nav_msgs sensor_msgs
    visualization_msgs tf message_filters cv_bridge pcl_ros pcl_conversions
    motion_planner_log pct_planner message_runtime
  DEPENDS EIGEN3 PCL OpenCV Boost)

# 保留原目录结构的 include 路径，同时加入 local_sensing 的第三方头文件路径。
set(LOCAL_SENSING_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/local_sensing)
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../path_searching/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/Utils/pose_utils/include
  ${LOCAL_SENSING_DIR}/include
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} ${ARMADILLO_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# 规划内部库：目标名称保持不变，但不再是独立 ROS 包。
add_library(plan_env
  ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/src/grid_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/src/pct_terrain_map.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/src/raycast.cpp)
target_link_libraries(plan_env ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(path_searching ${CMAKE_CURRENT_SOURCE_DIR}/../path_searching/src/dyn_a_star.cpp)
target_link_libraries(path_searching ${catkin_LIBRARIES})

add_library(bspline_opt
  ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/src/uniform_bspline.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/src/bspline_optimizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/src/gradient_descent_optimizer.cpp)
target_link_libraries(bspline_opt ${catkin_LIBRARIES} path_searching plan_env)

add_library(traj_utils
  ${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/src/planning_visualization.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/src/polynomial_traj.cpp)
target_link_libraries(traj_utils ${catkin_LIBRARIES} bspline_opt)

add_library(pose_utils ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/Utils/pose_utils/src/pose_utils.cpp)
target_link_libraries(pose_utils ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

set(SCAN_TARGETS)

# SCAN 主节点及控制器。
add_executable(scan_planner_node src/scan_planner_node.cpp src/scan_replan_fsm.cpp src/planner_manager.cpp src/reference_path_z.cpp)
target_link_libraries(scan_planner_node ${catkin_LIBRARIES} plan_env path_searching bspline_opt traj_utils)
list(APPEND SCAN_TARGETS scan_planner_node)
add_executable(open_loop_controller src/open_loop_controller.cpp)
target_link_libraries(open_loop_controller ${catkin_LIBRARIES} bspline_opt)
list(APPEND SCAN_TARGETS open_loop_controller)
add_executable(closed_loop_controller src/closed_loop_controller.cpp)
target_link_libraries(closed_loop_controller ${catkin_LIBRARIES} bspline_opt)
list(APPEND SCAN_TARGETS closed_loop_controller)

# MPC/acados：仅消费已有自动生成源码，不安装 mpc_generated 源码目录。
set(ACADOS_ROOT ${PROJECT_SOURCE_DIR}/../../../../third_party/acados)
set(ACADOS_INSTALL ${ACADOS_ROOT}/install)
set(MPC_GENERATED_DIR ${PROJECT_SOURCE_DIR}/mpc_generated)
if(NOT EXISTS ${MPC_GENERATED_DIR}/acados_solver_scan_planar_mpc.c OR NOT EXISTS ${ACADOS_INSTALL}/lib/libacados.so)
  message(FATAL_ERROR "acados runtime or generated MPC solver is missing")
endif()
file(GLOB MPC_GENERATED_SOURCES ${MPC_GENERATED_DIR}/*.c ${MPC_GENERATED_DIR}/scan_planar_mpc_model/*.c ${MPC_GENERATED_DIR}/scan_planar_mpc_cost/*.c)
list(REMOVE_ITEM MPC_GENERATED_SOURCES ${MPC_GENERATED_DIR}/main_scan_planar_mpc.c ${MPC_GENERATED_DIR}/main_sim_scan_planar_mpc.c)
add_executable(mpc_controller src/mpc_controller.cpp ${MPC_GENERATED_SOURCES})
target_include_directories(mpc_controller PRIVATE ${MPC_GENERATED_DIR} ${MPC_GENERATED_DIR}/scan_planar_mpc_model ${MPC_GENERATED_DIR}/scan_planar_mpc_cost ${ACADOS_INSTALL}/include ${ACADOS_INSTALL}/include/acados ${ACADOS_INSTALL}/include/blasfeo/include ${ACADOS_INSTALL}/include/hpipm/include)
target_link_directories(mpc_controller PRIVATE ${ACADOS_INSTALL}/lib)
target_link_libraries(mpc_controller ${catkin_LIBRARIES} bspline_opt acados hpipm blasfeo qpOASES_e dl m)
set_target_properties(mpc_controller PROPERTIES BUILD_RPATH "${ACADOS_INSTALL}/lib" INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/acados/lib" LINK_FLAGS "-Wl,--disable-new-dtags")
list(APPEND SCAN_TARGETS mpc_controller)

add_executable(go2_kinematic_sim src/go2_kinematic_sim.cpp)
target_link_libraries(go2_kinematic_sim ${catkin_LIBRARIES})
list(APPEND SCAN_TARGETS go2_kinematic_sim)
add_executable(go2_gait_publisher src/go2_gait_publisher.cpp)
target_link_libraries(go2_gait_publisher ${catkin_LIBRARIES})
list(APPEND SCAN_TARGETS go2_gait_publisher)

# 原仿真工具目标，源码继续使用原目录。
add_executable(map_pub ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/map_generator/src/map_publisher.cpp)
target_link_libraries(map_pub ${catkin_LIBRARIES} ${PCL_LIBRARIES})
list(APPEND SCAN_TARGETS map_pub)
add_executable(mockamap_node
  ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/src/ces_randommap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/src/maps.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/src/mockamap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/src/perlinnoise.cpp)
target_include_directories(mockamap_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/include)
target_link_libraries(mockamap_node ${catkin_LIBRARIES})
list(APPEND SCAN_TARGETS mockamap_node)
add_executable(odom_visualization ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/Utils/odom_visualization/src/odom_visualization.cpp)
target_link_libraries(odom_visualization ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES} pose_utils)
list(APPEND SCAN_TARGETS odom_visualization)
add_executable(waypoint_generator ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/Utils/waypoint_generator/src/waypoint_generator.cpp)
target_link_libraries(waypoint_generator ${catkin_LIBRARIES})
list(APPEND SCAN_TARGETS waypoint_generator)

add_executable(pcl_render_node
  ${LOCAL_SENSING_DIR}/src/pointcloud_render_node.cpp
  ${LOCAL_SENSING_DIR}/include/ikd-Tree/ikd_Tree.cpp
  ${LOCAL_SENSING_DIR}/include/FOV_Checker/FOV_Checker.cpp)
target_compile_options(pcl_render_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(pcl_render_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${OpenMP_CXX_FLAGS})
list(APPEND SCAN_TARGETS pcl_render_node)

if(USE_GPU)
  add_library(GLAD ${LOCAL_SENSING_DIR}/include/glad.c)
  add_executable(opengl_render_node
    ${LOCAL_SENSING_DIR}/src/opengl_render_node.cpp
    ${LOCAL_SENSING_DIR}/include/ikd-Tree/ikd_Tree.cpp
    ${LOCAL_SENSING_DIR}/include/FOV_Checker/FOV_Checker.cpp)
  target_include_directories(opengl_render_node PRIVATE ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS} ${glfw3_INCLUDE_DIRS})
  if(NOT glfw3_LIBS)
    set(glfw3_LIBS glfw)
  endif()
  target_link_libraries(opengl_render_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${glfw3_LIBS} GLAD)
  list(APPEND SCAN_TARGETS opengl_render_node)
endif()
add_dependencies(${SCAN_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 统一安装所有运行目标和原目录中的必要资源，不安装第三方源码和测试目录。
install(TARGETS ${SCAN_TARGETS} plan_env path_searching bspline_opt traj_utils pose_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../path_searching/include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bspline_opt/include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../traj_utils/include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/Utils/pose_utils/include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
endif()
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/mockamap/config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/mockamap PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../simulator/Utils/odom_visualization/meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
install(DIRECTORY ${LOCAL_SENSING_DIR}/params/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY ${LOCAL_SENSING_DIR}/include/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/local_sensing FILES_MATCHING PATTERN "*.vs" PATTERN "*.fs")
install(DIRECTORY scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts FILES_MATCHING PATTERN "*.py" PATTERN "*.sh" PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
file(GLOB ACADOS_RUNTIME_LIBS "${ACADOS_INSTALL}/lib/*.so*")
if(ACADOS_RUNTIME_LIBS)
  install(FILES ${ACADOS_RUNTIME_LIBS} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/acados/lib)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(reference_path_z_test test/reference_path_z_test.cpp src/reference_path_z.cpp)
  if(TARGET reference_path_z_test)
    target_link_libraries(reference_path_z_test ${catkin_LIBRARIES})
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/test/pct_terrain_map_test.cpp")
    catkin_add_gtest(pct_terrain_map_test ${CMAKE_CURRENT_SOURCE_DIR}/../plan_env/test/pct_terrain_map_test.cpp)
    if(TARGET pct_terrain_map_test)
      target_link_libraries(pct_terrain_map_test ${catkin_LIBRARIES} plan_env)
    endif()
  endif()
endif()
