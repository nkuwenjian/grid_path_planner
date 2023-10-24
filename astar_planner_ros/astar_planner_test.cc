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

#include <memory>
#include <mutex>  // NOLINT

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "glog/logging.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"

#include "astar_planner_ros/common/utils.h"
#include "astar_planner_ros/grid_search/grid_search.h"

namespace astar_planner_ros {

namespace {
constexpr uint8_t kOccupied = 100;
constexpr uint8_t kFree = 0;
}  // namespace

class AStarPlannerTest {
 public:
  AStarPlannerTest();
  virtual ~AStarPlannerTest() = default;

 private:
  void SetMap(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void SetStart(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
  void SetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal);
  void MakePlan();
  static void PublishPlan(const std::vector<geometry_msgs::PoseStamped>& plan,
                          const ros::Publisher& pub);

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;

  uint32_t last_size_x_ = 0;
  uint32_t last_size_y_ = 0;
  uint32_t size_x_ = 0;
  uint32_t size_y_ = 0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  float resolution_ = 0.0;
  std::vector<std::vector<uint8_t>> map_;
  std::mutex mutex_;

  int start_x_ = 0;
  int start_y_ = 0;
  int goal_x_ = 0;
  int goal_y_ = 0;
  bool start_received_ = false;
  bool goal_received_ = false;
  std::string global_frame_;
  ros::Publisher plan_pub_;

  std::unique_ptr<grid_search::GridSearch> planner_ = nullptr;
};

AStarPlannerTest::AStarPlannerTest() {
  map_sub_ = nh_.subscribe("map", 1, &AStarPlannerTest::SetMap, this);
  start_sub_ =
      nh_.subscribe("initialpose", 1, &AStarPlannerTest::SetStart, this);
  goal_sub_ = nh_.subscribe("move_base_simple/goal", 1,
                            &AStarPlannerTest::SetGoal, this);
  plan_pub_ = nh_.advertise<nav_msgs::Path>("grid_path", 1);
}

void AStarPlannerTest::SetMap(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  std::lock_guard<std::mutex> lock(mutex_);
  size_x_ = map->info.width;
  size_y_ = map->info.height;
  resolution_ = map->info.resolution;
  origin_x_ = map->info.origin.position.x;
  origin_y_ = map->info.origin.position.y;

  // http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
  // The map data, in row-major order, starting with (0,0). Occupancy
  // probabilities are in the range[0, 100]. Unknown is - 1.
  // Here, we treat the unknown state as the occupied state.
  map_.clear();
  map_.resize(size_x_);
  for (std::size_t i = 0U; i < size_x_; ++i) {
    map_[i].resize(size_y_);
    for (uint32_t j = 0U; j < size_y_; ++j) {
      uint32_t index = i + j * size_x_;
      if (map->data[index] == kFree) {
        map_[i][j] = kFree;
      } else {
        map_[i][j] = kOccupied;
      }
    }
  }
  LOG(INFO) << "Map is received.";
}

void AStarPlannerTest::SetStart(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start) {
  int start_x =
      common::ContXY2Disc(start->pose.pose.position.x - origin_x_, resolution_);
  int start_y =
      common::ContXY2Disc(start->pose.pose.position.y - origin_y_, resolution_);

  if (start_x == start_x_ && start_y == start_y_) {
    return;
  }
  start_x_ = start_x;
  start_y_ = start_y;
  start_received_ = true;
  if (global_frame_.empty()) {
    global_frame_ = start->header.frame_id;
  } else {
    CHECK_EQ(global_frame_, start->header.frame_id);
  }
  LOG(INFO) << "A new start (" << start_x_ << ", " << start_y_
            << ") is received.";

  MakePlan();
}

void AStarPlannerTest::SetGoal(
    const geometry_msgs::PoseStamped::ConstPtr& goal) {
  // retrieving goal position
  int goal_x =
      common::ContXY2Disc(goal->pose.position.x - origin_x_, resolution_);
  int goal_y =
      common::ContXY2Disc(goal->pose.position.y - origin_y_, resolution_);

  if (goal_x == goal_x_ && goal_y == goal_y_) {
    return;
  }
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  goal_received_ = true;
  if (global_frame_.empty()) {
    global_frame_ = goal->header.frame_id;
  } else {
    CHECK_EQ(global_frame_, goal->header.frame_id);
  }
  LOG(INFO) << "A new goal (" << goal_x_ << ", " << goal_y_ << ") is received.";

  MakePlan();
}

void AStarPlannerTest::MakePlan() {
  // if a start as well as goal are defined go ahead and plan
  if (!start_received_ || !goal_received_) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (planner_ == nullptr || last_size_x_ != size_x_ ||
      last_size_y_ != size_y_) {
    planner_.reset();
    planner_ = std::make_unique<grid_search::GridSearch>(size_x_, size_y_,
                                                         resolution_);
    last_size_x_ = size_x_;
    last_size_y_ = size_y_;
  }

  grid_search::GridAStarResult result;
  if (!planner_->GenerateGridPath(start_x_, start_y_, goal_x_, goal_y_, map_,
                                  kOccupied, grid_search::SearchType::A_STAR,
                                  &result)) {
    LOG(ERROR) << "A-star search failed.";
    return;
  }
  LOG(INFO) << "A-star search successfully.";

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = ros::Time::now();
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  CHECK_EQ(result.x.size(), result.y.size());
  const std::size_t N = result.x.size();
  std::vector<geometry_msgs::PoseStamped> plan;
  for (std::size_t i = 0; i < N; i++) {
    pose.pose.position.x =
        common::DiscXY2Cont(result.x[i], resolution_) + origin_x_;
    pose.pose.position.y =
        common::DiscXY2Cont(result.y[i], resolution_) + origin_y_;
    plan.push_back(pose);
  }
  PublishPlan(plan, plan_pub_);
}

void AStarPlannerTest::PublishPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const ros::Publisher& pub) {
  if (plan.empty()) {
    return;
  }

  nav_msgs::Path gui_path;
  gui_path.header = plan.front().header;
  gui_path.poses = plan;
  pub.publish(gui_path);
}

}  // namespace astar_planner_ros

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "astar_planner_test");
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  astar_planner_ros::AStarPlannerTest astar_planner_test;
  ros::spin();

  return 0;
}
