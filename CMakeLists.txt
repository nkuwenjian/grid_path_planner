cmake_minimum_required(VERSION 3.0.2)
project(grid_path_planner)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/grid_path_planner;
#   -warnings-as-errors=*;
# )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  tf2_ros
)

find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
  LIBRARIES grid_path_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    tf2_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(grid_path_planner
  grid_path_planner/grid_path_planner_ros.cc
  grid_path_planner/grid_search/grid_search.cc
  grid_path_planner/common/heap.cc
)
target_link_libraries(grid_path_planner ${catkin_LIBRARIES} glog::glog)

add_executable(grid_path_planner_test grid_path_planner/grid_path_planner_ros_test.cc)
target_link_libraries(grid_path_planner_test grid_path_planner ${catkin_LIBRARIES})

add_executable(grid_path_planner_tf_broadcaster grid_path_planner/tf_broadcaster.cc)
target_link_libraries(grid_path_planner_tf_broadcaster ${catkin_LIBRARIES} glog::glog)
