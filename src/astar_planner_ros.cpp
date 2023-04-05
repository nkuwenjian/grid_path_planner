/******************************************************************************
 * Copyright (c) 2023, NKU Mobile & Flying Robotics Lab
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
 *
 * Author: Jian Wen (nkuwenjian@gmail.com)
 *****************************************************************************/

#include "astar_planner_ros/astar_planner_ros.h"

#include "costmap_2d/cost_values.h"
#include "costmap_2d/inflation_layer.h"
#include "glog/logging.h"
#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"

PLUGINLIB_EXPORT_CLASS(astar_planner_ros::AStarPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace astar_planner_ros {

AStarPlannerROS::AStarPlannerROS(const std::string& name,
                                 costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void AStarPlannerROS::initialize(std::string name,
                                 costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  VLOG(4) << "Name is " << name;
  name_ = name;
  costmap_ros_ = costmap_ros;
  circumscribed_cost_ = ComputeCircumscribedCost();
  if (circumscribed_cost_ == costmap_2d::LETHAL_OBSTACLE) {
    return;
  }

  astar_planner_ = std::make_unique<GridSearch>(
      costmap_ros_->getCostmap()->getSizeInCellsX(),
      costmap_ros_->getCostmap()->getSizeInCellsY(),
      costmap_ros_->getCostmap()->getResolution());

  plan_pub_ = private_nh.advertise<nav_msgs::Path>("grid_path", 1);
  initialized_ = true;
}

uint8_t AStarPlannerROS::ComputeCircumscribedCost() const {
  if (!costmap_ros_) {
    LOG(ERROR) << "Costmap is not initialized";
    return costmap_2d::LETHAL_OBSTACLE;
  }

  // check if the costmap has an inflation layer
  const std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins =
      costmap_ros_->getLayeredCostmap()->getPlugins();
  boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = nullptr;
  uint8_t cost = costmap_2d::LETHAL_OBSTACLE;
  for (auto layer = plugins->begin(); layer != plugins->end(); ++layer) {
    inflation_layer =
        boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
    if (inflation_layer == nullptr) {
      continue;
    }

    cost = inflation_layer->computeCost(
        costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() /
        costmap_ros_->getCostmap()->getResolution());
    break;
  }

  if (inflation_layer == nullptr) {
    LOG(ERROR) << "Failed to get inflation layer";
    return costmap_2d::LETHAL_OBSTACLE;
  }
  return cost;
}

bool AStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal,
                               std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    LOG(ERROR) << "AStarPlannerROS is not initialized";
    return false;
  }

  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();
  double resolution = costmap_ros_->getCostmap()->getResolution();
  unsigned char* charmap = costmap_ros_->getCostmap()->getCharMap();

  int start_x = CONTXY2DISC(start.pose.position.x - origin_x, resolution);
  int start_y = CONTXY2DISC(start.pose.position.y - origin_y, resolution);
  int end_x = CONTXY2DISC(goal.pose.position.x - origin_x, resolution);
  int end_y = CONTXY2DISC(goal.pose.position.y - origin_y, resolution);

  GridAStarResult result;
  CHECK_NOTNULL(astar_planner_);
  bool success = astar_planner_->GenerateGridPath(
      start_x, start_y, end_x, end_y, charmap, circumscribed_cost_,
      SearchType::A_STAR, &result);

  if (!success) {
    LOG(WARNING) << "Failed to find a grid path";
    return false;
  }

  plan.clear();
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = start.header.frame_id;
  pose.header.stamp = ros::Time::now();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  CHECK_EQ(result.x.size(), result.y.size());
  for (size_t i = 0; i < result.x.size(); i++) {
    pose.pose.position.x = DISCXY2CONT(result.x[i], resolution) + origin_x;
    pose.pose.position.y = DISCXY2CONT(result.y[i], resolution) + origin_y;
    plan.push_back(pose);
  }
  PublishGlobalPlan(plan);

  return true;
}

void AStarPlannerROS::PublishGlobalPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) const {
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();

  gui_path.poses = plan;
  plan_pub_.publish(gui_path);
}

}  // namespace astar_planner_ros
