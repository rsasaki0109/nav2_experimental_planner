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

#ifndef NAV2_RRT_PLANNER__RRT_CONNECT_PLANNER_HPP_
#define NAV2_RRT_PLANNER__RRT_CONNECT_PLANNER_HPP_

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

/// Bidirectional sampling-based global planner (RRT-Connect) as a Nav2
/// nav2_core::GlobalPlanner. Upstream Nav2 ships only search-based global
/// planners (NavFn, Smac A*/Hybrid/Lattice, Theta*); it has no sampling-based
/// planner. RRT-Connect grows two trees — one rooted at the start, one at the
/// goal — and on each iteration extends one tree toward a random sample and then
/// greedily "connects" the other tree toward the new node. The greedy connect
/// step makes it markedly faster than plain RRT/RRT* at threading narrow
/// passages, at the cost of path optimality (RRT-Connect is feasible, not
/// asymptotically optimal — use RRTStarPlanner when shortest-path matters).
/// Classical (non-learned) and deterministic for a fixed random_seed.
class RRTConnectPlanner : public nav2_core::GlobalPlanner
{
public:
  RRTConnectPlanner() = default;
  ~RRTConnectPlanner() override = default;

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
  };

  enum class Status
  {
    kReached,   // the new node landed exactly on the target
    kAdvanced,  // a node was added one step toward the target
    kTrapped    // the step was blocked (collision / out of bounds)
  };

  /// True if the world point is in-bounds and below the lethal cost threshold.
  bool pointFree(double wx, double wy) const;

  /// True if the straight segment a->b stays collision-free (sampled at the
  /// costmap resolution).
  bool edgeFree(double ax, double ay, double bx, double by) const;

  /// One RRT extension of `tree` toward (tx, ty): steer from the nearest node by
  /// at most step_size_, add the node if the edge is free, and report whether
  /// the target was reached / advanced toward / blocked. On a non-trapped
  /// result `out_id` is the index of the newly added node.
  Status extend(std::vector<Node> & tree, double tx, double ty, int & out_id) const;

  /// Repeated extend(): greedily grow `tree` toward (tx, ty) until it reaches the
  /// target or gets trapped. Returns kReached or kTrapped.
  Status connect(std::vector<Node> & tree, double tx, double ty, int & out_id) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("RRTConnectPlanner")};

  // Parameters
  int max_iterations_{4000};
  double step_size_{0.5};        // max edge length when steering [m]
  double interpolation_resolution_{0.05};  // edge collision-check step [m]
  bool allow_unknown_{true};
  int random_seed_{1};
};

}  // namespace nav2_rrt_planner

#endif  // NAV2_RRT_PLANNER__RRT_CONNECT_PLANNER_HPP_
