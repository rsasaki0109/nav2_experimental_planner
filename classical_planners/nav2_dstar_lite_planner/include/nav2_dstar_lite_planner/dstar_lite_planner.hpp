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

#ifndef NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_dstar_lite_planner
{

/// D* Lite incremental-search global planner as a Nav2 nav2_core::GlobalPlanner.
/// Upstream Nav2 ships only one-shot search planners (NavFn, Smac, Theta*) that
/// replan from scratch every cycle; it has no incremental planner. D* Lite
/// (Koenig & Likhachev, 2002) searches the 8-connected costmap grid backward
/// from the goal, caching g / rhs values, and on the next plan only repairs the
/// vertices whose edge costs changed (plus a priority-key shift `km` for the
/// moved robot) instead of recomputing the whole field — far cheaper when the
/// costmap changes little between cycles (newly seen obstacles, dynamic agents).
/// Classical (non-learned) and fully deterministic, so it is testable.
class DStarLitePlanner : public nav2_core::GlobalPlanner
{
public:
  DStarLitePlanner() = default;
  ~DStarLitePlanner() override = default;

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
  struct Key
  {
    double k1{0.0};
    double k2{0.0};
  };

  struct QEntry
  {
    Key key;
    int idx{-1};
  };

  static bool keyLess(const Key & a, const Key & b);

  struct QGreater
  {
    bool operator()(const QEntry & a, const QEntry & b) const;
  };

  // --- grid helpers ---
  int index(int mx, int my) const {return my * width_ + mx;}
  bool blocked(int idx) const;
  double normCost(int idx) const;
  /// Edge traversal cost between two adjacent cells (inf if either is blocked).
  double edgeCost(int a, int b) const;
  /// Octile/Euclidean heuristic (in cell units) between two cells.
  double heuristic(int a, int b) const;
  std::vector<int> neighbours(int idx) const;

  // --- D* Lite core ---
  Key calcKey(int idx) const;
  double recomputeRhs(int idx) const;
  void insertOrUpdate(int idx, const Key & key);
  void removeFromQueue(int idx);
  bool topValid(Key & key, int & idx);
  void updateVertex(int idx);
  bool computeShortestPath(const std::function<bool()> & cancel_checker);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("DStarLitePlanner")};

  // Parameters
  double cost_weight_{3.0};       // how strongly to avoid high-cost cells
  double heuristic_weight_{1.0};  // <=1 keeps the heuristic admissible
  bool allow_unknown_{true};

  // Persistent D* Lite state (kept across createPlan calls for incremental repair)
  bool initialized_{false};
  int width_{0};
  int height_{0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double resolution_{0.05};
  int start_idx_{-1};
  int goal_idx_{-1};
  int last_start_idx_{-1};
  double km_{0.0};
  std::vector<double> g_;
  std::vector<double> rhs_;
  std::vector<unsigned char> snapshot_;
  std::priority_queue<QEntry, std::vector<QEntry>, QGreater> open_;
  std::unordered_map<int, Key> in_queue_;
};

}  // namespace nav2_dstar_lite_planner

#endif  // NAV2_DSTAR_LITE_PLANNER__DSTAR_LITE_PLANNER_HPP_
