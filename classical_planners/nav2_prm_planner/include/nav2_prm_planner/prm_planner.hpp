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

#ifndef NAV2_PRM_PLANNER__PRM_PLANNER_HPP_
#define NAV2_PRM_PLANNER__PRM_PLANNER_HPP_

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

namespace nav2_prm_planner
{

/// Probabilistic Roadmap (PRM) global planner as a Nav2 nav2_core::GlobalPlanner.
/// Upstream Nav2 ships only search-based global planners (NavFn, Smac
/// A*/Hybrid/Lattice, Theta*); it has no sampling-based roadmap planner. PRM
/// samples collision-free milestones over the global costmap, connects nearby
/// milestones with collision-free edges into an undirected graph, splices in the
/// start and goal, and returns the shortest path found by Dijkstra search over
/// the roadmap. Classical (non-learned) and deterministic for a fixed
/// random_seed so it is testable. Where RRT/RRT-Connect grow a tree toward a
/// single query, PRM builds a reusable graph of the free space; here the roadmap
/// is rebuilt per query because the costmap changes between plans.
class PRMPlanner : public nav2_core::GlobalPlanner
{
public:
  PRMPlanner() = default;
  ~PRMPlanner() override = default;

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
  struct Milestone
  {
    double x{0.0};
    double y{0.0};
  };

  struct Edge
  {
    int to{-1};
    double weight{0.0};
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
  rclcpp::Logger logger_{rclcpp::get_logger("PRMPlanner")};

  // Parameters
  int num_samples_{500};         // collision-free milestones to sample
  double connection_radius_{1.5};  // max edge length when wiring the graph [m]
  int max_neighbours_{10};       // max edges per milestone (k-nearest cap)
  double interpolation_resolution_{0.05};  // edge collision-check step [m]
  bool allow_unknown_{true};
  int random_seed_{1};
};

}  // namespace nav2_prm_planner

#endif  // NAV2_PRM_PLANNER__PRM_PLANNER_HPP_
