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

#ifndef NAV2_ARA_STAR_PLANNER__ARA_STAR_PLANNER_HPP_
#define NAV2_ARA_STAR_PLANNER__ARA_STAR_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_ara_star_planner
{

/// ARA* (Anytime Repairing A*) global planner as a Nav2 nav2_core::GlobalPlanner.
/// ARA* (Likhachev, Gordon & Thrun, 2003) runs a series of weighted-A* searches
/// with a shrinking inflation factor epsilon: the first search (large epsilon) is
/// cheap and returns a path guaranteed within epsilon of optimal; each later
/// search lowers epsilon and *reuses* the previous search effort (via the OPEN
/// and INCONS lists) to improve the path, tightening the suboptimality bound
/// toward 1 (optimal). It is therefore anytime — it produces a valid bounded-
/// suboptimal path quickly and keeps improving until epsilon reaches 1 or the
/// per-plan expansion budget runs out. Upstream Nav2 has no anytime / bounded-
/// suboptimal planner. Classical (non-learned) and fully deterministic.
class ARAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  ARAStarPlanner() = default;
  ~ARAStarPlanner() override = default;

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
  bool isFree(int idx) const;
  double normCost(int idx) const;
  /// Edge cost between two adjacent cells (inf if either is blocked).
  double edgeCost(int a, int b) const;
  /// Octile heuristic (in cell units) from a cell to the goal cell.
  double heuristic(int idx) const;
  std::vector<int> neighbours(int idx) const;
  /// One weighted-A* pass at the current inflation factor, reusing OPEN/INCONS.
  /// Returns false if the expansion budget was hit mid-pass.
  bool improvePath(double epsilon, const std::function<bool()> & cancel_checker);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("ARAStarPlanner")};

  // Parameters
  int lethal_threshold_{253};   // costs >= this are obstacles (default INSCRIBED)
  bool allow_unknown_{true};
  double cost_weight_{0.0};      // >0 biases away from high-cost cells
  double initial_epsilon_{3.0};  // starting inflation factor (>= 1)
  double epsilon_decrement_{0.5};  // epsilon step between anytime iterations
  int max_iterations_{200000};  // total expansion budget across all passes

  // Per-query search state.
  int width_{0};
  int height_{0};
  int start_idx_{-1};
  int goal_idx_{-1};
  int expansions_{0};
  std::vector<double> g_;
  std::vector<int> parent_;
  std::vector<char> closed_;
  std::vector<char> in_open_;
  std::vector<char> in_incons_;
  std::vector<int> incons_;
};

}  // namespace nav2_ara_star_planner

#endif  // NAV2_ARA_STAR_PLANNER__ARA_STAR_PLANNER_HPP_
