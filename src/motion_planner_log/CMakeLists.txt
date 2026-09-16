cmake_minimum_required(VERSION 2.8.3)
project(motion_planner_log)

######################
### 编译配置
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -pthread")

######################
### 依赖包
######################
find_package(catkin REQUIRED COMPONENTS roscpp roslib)
# Prefer the system Google glog. The fallback is only used when the system
# package is unavailable and the pinned source has already been provided.
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG QUIET libglog)

if(NOT GLOG_FOUND)
  set(MOTION_PLANNER_GLOG_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../third_party/glog")
  if(NOT EXISTS "${MOTION_PLANNER_GLOG_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "Google glog 0.4.0 is not installed. Run ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ensure_glog.sh "
      "to download the pinned fallback source, then rebuild.")
  endif()
  add_subdirectory("${MOTION_PLANNER_GLOG_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/motion_planner_glog" EXCLUDE_FROM_ALL)
  set(GLOG_INCLUDE_DIRS "${MOTION_PLANNER_GLOG_SOURCE_DIR}/src")
  set(GLOG_LIBRARIES glog)
endif()

######################
### Catkin 配置
######################
get_filename_component(MOTION_PLANNER_PROJECT_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motion_planner_log
  CATKIN_DEPENDS roscpp roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

# Keep the default log directory relative to the workspace source root. The
# runtime logger still applies its environment-variable and fallback policy.
add_definitions(-DMOTION_PLANNER_DEFAULT_LOG_DIR="${MOTION_PLANNER_PROJECT_ROOT}/log")

######################
### 日志库
######################
add_library(motion_planner_log src/logging.cpp)
target_compile_options(motion_planner_log PRIVATE ${GLOG_CFLAGS_OTHER})
target_link_libraries(motion_planner_log
  ${catkin_LIBRARIES}
  ${GLOG_LIBRARIES}
  pthread
)

#############
## 安装规则 ##
#############
# Install only the logger implementation and tooling. Runtime log files are
# intentionally kept in the workspace-level log/ directory and are never
# copied into the install space.
install(TARGETS motion_planner_log
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
install(DIRECTORY src/motion_planner_log/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)
install(PROGRAMS scripts/ensure_glog.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
