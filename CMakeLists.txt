cmake_minimum_required(VERSION 3.0.2)
project(astar_planner)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "BUILD TYPE: " ${CMAKE_BUILD_TYPE})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES astar_planner
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

add_library(${PROJECT_NAME} src/astar_planner_ros.cpp src/astar_planner.cpp src/heap.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})