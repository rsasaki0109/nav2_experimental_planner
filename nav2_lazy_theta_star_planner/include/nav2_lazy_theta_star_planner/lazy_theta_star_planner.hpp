// Copyright 2026 nav2_experimental_planner contributors
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

#ifndef NAV2_LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_HPP_
#define NAV2_LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_HPP_

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

namespace nav2_lazy_theta_star_planner
{

/// Lazy Theta* any-angle global planner as a Nav2 nav2_core::GlobalPlanner.
/// Theta* (Nash et al.) lets a node's parent be any earlier node visible to it,
/// so paths are not confined to the 8 grid directions — they bend at obstacle
/// corners and run straight elsewhere. Lazy Theta* (Nash, Koenig & Tovey, 2010)
/// is the variant that *defers* the line-of-sight check: it optimistically
/// assumes a successor is visible from its grandparent and only verifies (and
/// repairs) that assumption when the node is expanded, cutting the number of
/// line-of-sight checks from one-per-edge to about one-per-expanded-node.
/// Upstream Nav2 ships standard (eager) Theta* in nav2_theta_star_planner; this
/// adds the lazy variant, which is a distinct algorithm not in upstream. Treats
/// the costmap as a binary free/blocked grid (cells >= `lethal_threshold` are
/// obstacles). Classical (non-learned) and fully deterministic, so it is testable.
class LazyThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  LazyThetaStarPlanner() = default;
  ~LazyThetaStarPlanner() override = default;

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
  /// Straight-line (any-angle) cost between two cells, in cell units.
  double dist(int ax, int ay, int bx, int by) const;
  /// Grid line-of-sight between two cells (Theta* Bresenham-style cell check).
  bool lineOfSight(int ax, int ay, int bx, int by) const;
  std::vector<int> neighbours(int idx) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("LazyThetaStarPlanner")};

  // Parameters
  int lethal_threshold_{253};  // costs >= this are obstacles (default INSCRIBED)
  bool allow_unknown_{true};
  double heuristic_weight_{1.0};           // <=1 keeps the heuristic admissible
  double interpolation_resolution_{0.05};  // path densification step [m]

  // Per-query grid context (set at the top of createPlan).
  int width_{0};
  int height_{0};
};

}  // namespace nav2_lazy_theta_star_planner

#endif  // NAV2_LAZY_THETA_STAR_PLANNER__LAZY_THETA_STAR_PLANNER_HPP_
