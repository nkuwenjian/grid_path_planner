/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "astar_planner/astar_planner_ros.h"

#include "costmap_2d/inflation_layer.h"
#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"

PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace astar_planner {

AStarPlannerROS::AStarPlannerROS(std::string name,
                                 costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void AStarPlannerROS::initialize(std::string name,
                                 costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);

    ROS_DEBUG("Name is %s", name.c_str());

    name_ = name;
    costmap_ros_ = costmap_ros;

    circumscribed_cost_ = computeCircumscribedCost();

    astar_planner_ = std::make_unique<SBPL2DGridSearch>(
        costmap_ros_->getCostmap()->getSizeInCellsX(),
        costmap_ros_->getCostmap()->getSizeInCellsY(),
        costmap_ros_->getCostmap()->getResolution());

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("grid_path", 1);

    initialized_ = true;
  }
}

unsigned char AStarPlannerROS::computeCircumscribedCost() {
  unsigned char result = 0;

  if (!costmap_ros_) {
    ROS_ERROR("Costmap is not initialized");
    return 0;
  }

  // check if the costmap has an inflation layer
  for (std::vector<boost::shared_ptr<costmap_2d::Layer>>::const_iterator layer =
           costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
       ++layer) {
    boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer =
        boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);

    if (!inflation_layer) {
      continue;
    }

    result = inflation_layer->computeCost(
        costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() /
        costmap_ros_->getCostmap()->getResolution());
  }

  return result;
}

bool AStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("astar_planner is not initialized");
    return false;
  }

  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();
  double resolution = costmap_ros_->getCostmap()->getResolution();
  unsigned char* charmap = costmap_ros_->getCostmap()->getCharMap();

  int startx_c = CONTXY2DISC(start.pose.position.x - origin_x, resolution);
  int starty_c = CONTXY2DISC(start.pose.position.y - origin_y, resolution);
  int goalx_c = CONTXY2DISC(goal.pose.position.x - origin_x, resolution);
  int goaly_c = CONTXY2DISC(goal.pose.position.y - origin_y, resolution);

  std::vector<std::pair<int, int>> grid_path;
  bool success = astar_planner_->search(charmap, circumscribed_cost_, startx_c,
                                        starty_c, goalx_c, goaly_c, &grid_path);

  if (!success) {
    ROS_WARN("Failed to find a grid path");
    return false;
  }

  plan.clear();
  geometry_msgs::PoseStamped tmp;
  tmp.header.frame_id = start.header.frame_id;
  tmp.header.stamp = ros::Time::now();
  tmp.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  for (size_t i = 0; i < grid_path.size(); i++) {
    tmp.pose.position.x =
        DISCXY2CONT(grid_path[i].first, resolution) + origin_x;
    tmp.pose.position.y =
        DISCXY2CONT(grid_path[i].second, resolution) + origin_y;
    plan.push_back(tmp);
  }

  publishGlobalPlan(plan);

  return true;
}

void AStarPlannerROS::publishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();

  gui_path.poses = plan;
  plan_pub_.publish(gui_path);
}

}  // namespace astar_planner
