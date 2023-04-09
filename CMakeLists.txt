cmake_minimum_required(VERSION 3.0.2)
project(astar_planner_ros)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_ros
)

find_package(glog REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astar_planner_ros
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(astar_planner src/astar_planner.cpp src/heap.cpp)
target_link_libraries(astar_planner glog::glog)

add_executable(astar_planner_test src/astar_planner_test.cpp)
target_link_libraries(astar_planner_test astar_planner ${catkin_LIBRARIES})

add_library(astar_planner_ros src/astar_planner_ros.cpp)
target_link_libraries(astar_planner_ros astar_planner ${catkin_LIBRARIES})
