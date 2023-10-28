cmake_minimum_required(VERSION 3.0.2)
project(astar_planner_ros)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

# set(CMAKE_CXX_CLANG_TIDY
#   clang-tidy;
#   -header-filter=${PROJECT_SOURCE_DIR}/astar_planner_ros;
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
  LIBRARIES astar_planner_ros
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

add_library(grid_search astar_planner_ros/grid_search/grid_search.cc astar_planner_ros/common/heap.cc)
target_link_libraries(grid_search glog::glog)

add_library(astar_planner_ros astar_planner_ros/astar_planner_ros.cc)
target_link_libraries(astar_planner_ros grid_search ${catkin_LIBRARIES})

add_executable(astar_planner_ros_test astar_planner_ros/astar_planner_ros_test.cc)
target_link_libraries(astar_planner_ros_test astar_planner_ros ${catkin_LIBRARIES})

add_executable(astar_planner_tf_broadcaster astar_planner_ros/tf_broadcaster.cc)
target_link_libraries(astar_planner_tf_broadcaster ${catkin_LIBRARIES} glog::glog)
