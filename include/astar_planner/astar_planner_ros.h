#ifndef _ASTAR_PLANNER_ASTAR_PLANNER_ROS_H_
#define _ASTAR_PLANNER_ASTAR_PLANNER_ROS_H_

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// global representation
#include <nav_core/base_global_planner.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

#include "astar_planner/astar_planner.h"

namespace astar_planner
{
class AStarPlannerROS : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief  Default constructor for the AStarPlannerROS object
   */
  AStarPlannerROS();

  /**
   * @brief  Constructor for the AStarPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  AStarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the AStarPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~AStarPlannerROS();

private:
  unsigned char computeCircumscribedCost();
  void publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

private:
  SBPL2DGridSearch* astar_planner_;
  bool initialized_;

  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_;  //!< manages the cost map for us
  unsigned char circumscribed_cost_;

  ros::Publisher plan_pub_;
};

}  // namespace astar_planner

#endif  // _ASTAR_PLANNER_ASTAR_PLANNER_ROS_H_