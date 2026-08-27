cmake_minimum_required(VERSION 2.8.3)
project(scan_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
  visualization_msgs
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  DataDisp.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scan_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(scan_planner_node
  src/scan_planner_node.cpp 
  src/scan_replan_fsm.cpp
  src/planner_manager.cpp
  src/reference_path_z.cpp
  )
target_link_libraries(scan_planner_node 
  ${catkin_LIBRARIES}
  )
add_dependencies(scan_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(open_loop_controller src/open_loop_controller.cpp)
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
add_dependencies(open_loop_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(closed_loop_controller src/closed_loop_controller.cpp)
target_link_libraries(closed_loop_controller ${catkin_LIBRARIES})
add_dependencies(closed_loop_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

set(ACADOS_ROOT ${PROJECT_SOURCE_DIR}/../../../../third_party/acados)
set(ACADOS_INSTALL ${ACADOS_ROOT}/install)
set(MPC_GENERATED_DIR ${PROJECT_SOURCE_DIR}/mpc_generated)
if(NOT EXISTS ${MPC_GENERATED_DIR}/acados_solver_scan_planar_mpc.c OR NOT EXISTS ${ACADOS_INSTALL}/lib/libacados.so)
  message(FATAL_ERROR "acados runtime or generated MPC solver is missing. Run scripts/generate_mpc_solver.py first.")
endif()
file(GLOB MPC_GENERATED_SOURCES ${MPC_GENERATED_DIR}/*.c ${MPC_GENERATED_DIR}/scan_planar_mpc_model/*.c ${MPC_GENERATED_DIR}/scan_planar_mpc_cost/*.c)
list(REMOVE_ITEM MPC_GENERATED_SOURCES ${MPC_GENERATED_DIR}/main_scan_planar_mpc.c ${MPC_GENERATED_DIR}/main_sim_scan_planar_mpc.c)
add_executable(mpc_controller src/mpc_controller.cpp ${MPC_GENERATED_SOURCES})
target_include_directories(mpc_controller PRIVATE ${MPC_GENERATED_DIR} ${MPC_GENERATED_DIR}/scan_planar_mpc_model ${MPC_GENERATED_DIR}/scan_planar_mpc_cost ${ACADOS_INSTALL}/include ${ACADOS_INSTALL}/include/acados ${ACADOS_INSTALL}/include/blasfeo/include ${ACADOS_INSTALL}/include/hpipm/include)
target_link_directories(mpc_controller PRIVATE ${ACADOS_INSTALL}/lib)
target_link_libraries(mpc_controller ${catkin_LIBRARIES} acados hpipm blasfeo qpOASES_e dl m)
set_target_properties(mpc_controller PROPERTIES BUILD_RPATH "${ACADOS_INSTALL}/lib" INSTALL_RPATH "${ACADOS_INSTALL}/lib" LINK_FLAGS "-Wl,--disable-new-dtags")
add_dependencies(mpc_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(go2_kinematic_sim src/go2_kinematic_sim.cpp)
target_link_libraries(go2_kinematic_sim ${catkin_LIBRARIES})

add_executable(go2_gait_publisher src/go2_gait_publisher.cpp)
target_link_libraries(go2_gait_publisher ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(reference_path_z_test
    test/reference_path_z_test.cpp
    src/reference_path_z.cpp)
  if(TARGET reference_path_z_test)
    target_link_libraries(reference_path_z_test ${catkin_LIBRARIES})
  endif()
endif()
