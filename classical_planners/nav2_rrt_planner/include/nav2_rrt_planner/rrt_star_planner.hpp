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

#ifndef NAV2_RRT_PLANNER__RRT_STAR_PLANNER_HPP_
#define NAV2_RRT_PLANNER__RRT_STAR_PLANNER_HPP_

#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_rrt_planner
{

/// Asymptotically optimal sampling-based global planner (RRT*) as a Nav2
/// nav2_core::GlobalPlanner. Upstream Nav2 ships only search-based global
/// planners (NavFn, Smac A*/Hybrid/Lattice, Theta*); it has no sampling-based
/// planner. RRT* grows a tree of collision-free edges over the global costmap
/// with goal-biased sampling and neighbourhood rewiring, then returns the best
/// start->goal path found. Classical (non-learned), and deterministic for a
/// fixed random_seed so it is testable.
class RRTStarPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTStarPlanner() = default;
  ~RRTStarPlanner() override = default;

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
  struct Node
  {
    double x{0.0};
    double y{0.0};
    int parent{-1};
    double cost{0.0};  // path cost from the start through the tree
  };

  /// True if the world point is in-bounds and below the lethal cost threshold.
  bool pointFree(double wx, double wy) const;

  /// True if the straight segment a->b stays collision-free (sampled at the
  /// costmap resolution).
  bool edgeFree(double ax, double ay, double bx, double by) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RRTStarPlanner")};

  // Parameters
  int max_iterations_{4000};
  double step_size_{0.5};        // max edge length when steering [m]
  double goal_bias_{0.10};       // probability of sampling the goal
  double goal_tolerance_{0.25};  // reach radius for the goal [m]
  double rewire_radius_{1.0};    // neighbourhood radius for RRT* rewiring [m]
  double interpolation_resolution_{0.05};  // edge collision-check step [m]
  bool allow_unknown_{true};
  int random_seed_{1};
};

}  // namespace nav2_rrt_planner

#endif  // NAV2_RRT_PLANNER__RRT_STAR_PLANNER_HPP_
