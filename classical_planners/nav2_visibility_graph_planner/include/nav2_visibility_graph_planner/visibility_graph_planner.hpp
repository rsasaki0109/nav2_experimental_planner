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

#ifndef NAV2_VISIBILITY_GRAPH_PLANNER__VISIBILITY_GRAPH_PLANNER_HPP_
#define NAV2_VISIBILITY_GRAPH_PLANNER__VISIBILITY_GRAPH_PLANNER_HPP_

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

namespace nav2_visibility_graph_planner
{

/// Visibility-graph global planner as a Nav2 nav2_core::GlobalPlanner. Unlike
/// Nav2's grid planners (NavFn, Smac, Theta*) and the sampling planners in this
/// repo, a visibility graph reasons in continuous space about obstacle geometry:
/// the only places an obstacle-avoiding shortest path can turn are the convex
/// corners of obstacles, so the graph's vertices are those corners (plus the
/// start and goal) and its edges connect every mutually visible pair. A
/// shortest-path search (A*) over that graph yields a piecewise-straight,
/// corner-hugging route. Here the corners are extracted from the costmap grid
/// (free cells sitting at a convex obstacle corner) and visibility is a
/// line-of-sight check, so the result is a grid approximation of the geometric
/// visibility graph. Upstream Nav2 has no visibility-graph planner. Classical
/// (non-learned) and fully deterministic.
class VisibilityGraphPlanner : public nav2_core::GlobalPlanner
{
public:
  VisibilityGraphPlanner() = default;
  ~VisibilityGraphPlanner() override = default;

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
  bool isFree(int x, int y) const;
  /// True if the free cell (x, y) sits at a convex corner of an obstacle.
  bool isConvexCorner(int x, int y) const;
  /// Grid line-of-sight between two cells, sampled at the densification step.
  bool lineOfSight(int ax, int ay, int bx, int by) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("VisibilityGraphPlanner")};

  // Parameters
  int lethal_threshold_{253};   // costs >= this are obstacles (default INSCRIBED)
  bool allow_unknown_{true};
  double interpolation_resolution_{0.05};  // path densification / LOS step [m]
  int max_corners_{1500};       // cap on graph corner vertices (O(V^2) edges)

  // Per-query grid context.
  int width_{0};
  int height_{0};
};

}  // namespace nav2_visibility_graph_planner

#endif  // NAV2_VISIBILITY_GRAPH_PLANNER__VISIBILITY_GRAPH_PLANNER_HPP_
