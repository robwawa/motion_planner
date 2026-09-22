cmake_minimum_required(VERSION 3.16)
project(pct_map_cleaner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(FLANN REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

add_library(pct_map_cleaner_core src/pct_map_cleaner.cpp)
target_include_directories(pct_map_cleaner_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${FLANN_INCLUDE_DIRS}
)
target_compile_options(pct_map_cleaner_core PRIVATE
  ${PCL_DEFINITIONS}
  -O3 -DNDEBUG -Wall -Wextra -Wpedantic
)
target_link_libraries(pct_map_cleaner_core
  PUBLIC
    ${PCL_LIBRARIES}
    ${FLANN_LIBRARIES}
    ${LZ4_LIBRARY}
    OpenMP::OpenMP_CXX
    yaml-cpp
)

add_executable(pct_map_cleaner src/main.cpp)
target_compile_definitions(pct_map_cleaner PRIVATE
  PCT_MAP_CLEANER_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/pct_map_cleaner.yaml"
)
target_link_libraries(pct_map_cleaner PRIVATE pct_map_cleaner_core)

add_executable(pct_map_cleaner_compare tools/compare_clouds.cpp)
target_link_libraries(pct_map_cleaner_compare PRIVATE pct_map_cleaner_core)
target_compile_options(pct_map_cleaner_compare PRIVATE -O3 -DNDEBUG -Wall -Wextra -Wpedantic)

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME pct_map_cleaner_help COMMAND pct_map_cleaner --help)

  add_library(pct_map_cleaner_reference STATIC tests/reference/reference_cleaner.cpp)
  target_link_libraries(pct_map_cleaner_reference PRIVATE pct_map_cleaner_core)
  target_compile_options(pct_map_cleaner_reference PRIVATE -O3 -DNDEBUG)
  add_executable(pct_map_cleaner_baseline tests/reference/reference_main.cpp)
  target_link_libraries(pct_map_cleaner_baseline PRIVATE pct_map_cleaner_reference)
  target_compile_definitions(pct_map_cleaner_baseline PRIVATE
    PCT_MAP_CLEANER_DEFAULT_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/pct_map_cleaner.yaml")

  add_executable(pct_map_cleaner_test tests/cleaner_test.cpp)
  target_link_libraries(pct_map_cleaner_test PRIVATE pct_map_cleaner_core pct_map_cleaner_reference)
  target_compile_options(pct_map_cleaner_test PRIVATE -O3 -DNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME pct_map_cleaner_algorithms COMMAND pct_map_cleaner_test)
  add_test(NAME pct_map_cleaner_cli_and_tools
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/tools_test.py
      $<TARGET_FILE:pct_map_cleaner> $<TARGET_FILE:pct_map_cleaner_compare>)
endif()
