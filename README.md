# astar_planner_ros

This repo contains a global planner plugin of a grid-based A-star search algorithm for ROS navigation stack.

## How to use?

### 1. Preliminary

Install Google glog. The version of v0.5.0 is tested.
```
$ sudo apt install autoconf automake libtool libunwind-dev
$ git clone https://github.com/google/glog.git
$ cd glog/
$ git checkout v0.5.0
$ mkdir build
$ cd build/
$ cmake ..
$ make -j4
$ sudo make install
```
### 2. Clone and build astar_planner_ros
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/nkuwenjian/astar_planner_ros.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 3. Setup the configurations for ROS navigation stack

In the move_base launch file, use astar_planner_ros to override the default global planner, which may like this:
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" >
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/costmap_common_params.yaml" command="load" ns="global_costmap" />
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/costmap_common_params.yaml" command="load" ns="local_costmap" />
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/global_costmap_params.yaml" command="load" />
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/local_costmap_params.yaml" command="load" />

  <param name="base_global_planner" value="astar_planner_ros/AStarPlannerROS" />
  <param name="planner_frequency" value="1.0" />
  <param name="planner_patience" value="5.0" />

  ...
</node>
```
