// Copyright 2026 Nav2PlannerBattle contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_JPS_PLANNER__JPS_PLANNER_HPP_
#define NAV2_JPS_PLANNER__JPS_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_jps_planner
{

/// Jump Point Search (JPS) global planner as a Nav2 nav2_core::GlobalPlanner.
/// JPS (Harabor & Grastien, 2011) is an optimal speed-up of grid A* on uniform-
/// cost grids: it exploits grid path symmetry to "jump" over chains of cells that
/// A* would expand one by one, pushing only jump points (turning points and
/// forced-neighbour cells) onto the open list. It returns the same optimal path
/// as 8-connected A* while expanding far fewer nodes. Upstream Nav2's grid
/// planners (NavFn, Smac) do not include JPS. Because JPS assumes a uniform-cost
/// grid, it treats the costmap as a binary free/blocked grid (cells at or above
/// `lethal_threshold` are obstacles) and ignores graded inflation costs — use
/// NavFn/Smac or D* Lite when soft cost shaping matters. Classical (non-learned)
/// and fully deterministic, so it is testable.
class JPSPlanner : public nav2_core::GlobalPlanner
{
public:
  JPSPlanner() = default;
  ~JPSPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  int index(int x, int y) const {return y * width_ + x;}
  /// True if (x, y) is in-bounds and not an obstacle.
  bool isFree(int x, int y) const;
  /// Octile distance (8-connected uniform-cost heuristic) between two cells.
  double octile(int ax, int ay, int bx, int by) const;
  /// Jump from (x, y) along (dx, dy) until a jump point / goal / obstacle.
  /// Returns the jump-point cell index, or -1 if the ray hits a wall first.
  int jump(int x, int y, int dx, int dy) const;
  /// Pruned set of search directions to expand from (x, y), given the unit
  /// direction (pdx, pdy) by which the node was reached (0,0 for the start).
  std::vector<std::pair<int, int>> prunedDirections(int x, int y, int pdx, int pdy) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("JPSPlanner")};

  // Parameters
  int lethal_threshold_{253};  // costs >= this are obstacles (default INSCRIBED)
  bool allow_unknown_{true};

  // Per-query grid context (set at the top of createPlan).
  int width_{0};
  int height_{0};
  int gx_{0};  // goal cell x
  int gy_{0};  // goal cell y
};

}  // namespace nav2_jps_planner

#endif  // NAV2_JPS_PLANNER__JPS_PLANNER_HPP_
