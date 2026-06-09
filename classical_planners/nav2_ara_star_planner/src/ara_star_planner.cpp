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

#include "nav2_ara_star_planner/ara_star_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_ara_star_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309514880;
}  // namespace

void ARAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();

  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node, name_ + ".lethal_threshold", rclcpp::ParameterValue(253));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name_ + ".cost_weight", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(
    node, name_ + ".initial_epsilon", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, name_ + ".epsilon_decrement", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(200000));

  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".cost_weight", cost_weight_);
  node->get_parameter(name_ + ".initial_epsilon", initial_epsilon_);
  node->get_parameter(name_ + ".epsilon_decrement", epsilon_decrement_);
  node->get_parameter(name_ + ".max_iterations", max_iterations_);

  RCLCPP_INFO(
    logger_, "ARAStarPlanner '%s' configured: initial_epsilon=%.2f decrement=%.2f",
    name_.c_str(), initial_epsilon_, epsilon_decrement_);
}

void ARAStarPlanner::cleanup() {}
void ARAStarPlanner::activate() {}
void ARAStarPlanner::deactivate() {}

bool ARAStarPlanner::isFree(int idx) const
{
  const unsigned char cost = costmap_->getCost(
    static_cast<unsigned int>(idx % width_), static_cast<unsigned int>(idx / width_));
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_threshold_;
}

double ARAStarPlanner::normCost(int idx) const
{
  const unsigned char cost = costmap_->getCost(
    static_cast<unsigned int>(idx % width_), static_cast<unsigned int>(idx / width_));
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return 0.0;
  }
  const double denom = static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  return static_cast<double>(cost) / denom;
}

double ARAStarPlanner::edgeCost(int a, int b) const
{
  if (!isFree(a) || !isFree(b)) {
    return kInf;
  }
  const int ax = a % width_;
  const int ay = a / width_;
  const int bx = b % width_;
  const int by = b / width_;
  const double base = (ax != bx && ay != by) ? kSqrt2 : 1.0;
  const double avg_norm = 0.5 * (normCost(a) + normCost(b));
  return base * (1.0 + cost_weight_ * avg_norm);
}

double ARAStarPlanner::heuristic(int idx) const
{
  const int dx = std::abs(idx % width_ - goal_idx_ % width_);
  const int dy = std::abs(idx / width_ - goal_idx_ / width_);
  const int dmin = std::min(dx, dy);
  const int dmax = std::max(dx, dy);
  return (dmax - dmin) + kSqrt2 * dmin;
}

std::vector<int> ARAStarPlanner::neighbours(int idx) const
{
  std::vector<int> out;
  out.reserve(8);
  const int x = idx % width_;
  const int y = idx / width_;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const int nx = x + dx;
      const int ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) {
        continue;
      }
      out.push_back(index(nx, ny));
    }
  }
  return out;
}

bool ARAStarPlanner::improvePath(double epsilon, const std::function<bool()> & cancel_checker)
{
  // Build a fresh priority queue from the current OPEN membership at this
  // epsilon (keys change when epsilon changes, so they are recomputed here).
  using QItem = std::pair<double, int>;  // (key, idx)
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
  for (int i = 0; i < static_cast<int>(in_open_.size()); ++i) {
    if (in_open_[i]) {
      open.push({g_[i] + epsilon * heuristic(i), i});
    }
  }

  while (!open.empty()) {
    // key(goal) = g[goal]; stop once the cheapest OPEN key is not better.
    if (open.top().first >= g_[goal_idx_]) {
      break;
    }
    const double top_key = open.top().first;
    const int u = open.top().second;
    open.pop();
    if (!in_open_[u] || top_key != g_[u] + epsilon * heuristic(u)) {
      continue;  // stale entry
    }
    in_open_[u] = 0;
    closed_[u] = 1;

    if (++expansions_ > max_iterations_) {
      return false;
    }
    if ((expansions_ & 0x3FF) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }

    for (const int n : neighbours(u)) {
      const double c = edgeCost(u, n);
      if (c == kInf) {
        continue;
      }
      const double ng = g_[u] + c;
      if (ng < g_[n]) {
        g_[n] = ng;
        parent_[n] = u;
        if (!closed_[n]) {
          in_open_[n] = 1;
          open.push({ng + epsilon * heuristic(n), n});
        } else if (!in_incons_[n]) {
          in_incons_[n] = 1;
          incons_.push_back(n);
        }
      }
    }
  }
  return true;
}

nav_msgs::msg::Path ARAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  costmap_ = costmap_ros_->getCostmap();

  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  auto ros_node = node_.lock();
  plan.header.stamp = ros_node ? ros_node->now() : rclcpp::Clock().now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerTFError(
            "ARAStarPlanner expects start/goal in the " + global_frame_ + " frame");
  }

  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;

  unsigned int smx = 0;
  unsigned int smy = 0;
  unsigned int gmx = 0;
  unsigned int gmy = 0;
  if (!costmap_->worldToMap(sx, sy, smx, smy)) {
    throw nav2_core::StartOccupied("Start pose is out of costmap bounds");
  }
  if (!costmap_->worldToMap(gx, gy, gmx, gmy)) {
    throw nav2_core::GoalOccupied("Goal pose is out of costmap bounds");
  }

  width_ = static_cast<int>(costmap_->getSizeInCellsX());
  height_ = static_cast<int>(costmap_->getSizeInCellsY());
  start_idx_ = index(static_cast<int>(smx), static_cast<int>(smy));
  goal_idx_ = index(static_cast<int>(gmx), static_cast<int>(gmy));

  if (!isFree(start_idx_)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal cell");
  }
  if (!isFree(goal_idx_)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal cell");
  }

  if (cancel_checker && cancel_checker()) {
    throw nav2_core::PlannerCancelled("Planning cancelled");
  }

  const std::size_t cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  g_.assign(cells, kInf);
  parent_.assign(cells, -1);
  closed_.assign(cells, 0);
  in_open_.assign(cells, 0);
  in_incons_.assign(cells, 0);
  incons_.clear();
  expansions_ = 0;

  g_[start_idx_] = 0.0;
  parent_[start_idx_] = start_idx_;
  in_open_[start_idx_] = 1;

  double epsilon = std::max(1.0, initial_epsilon_);
  const double decrement = std::max(0.01, epsilon_decrement_);

  // First (cheap, epsilon-suboptimal) pass.
  improvePath(epsilon, cancel_checker);

  // Anytime improvement: lower epsilon, fold INCONS back into OPEN, re-open the
  // closed set, and search again, tightening the bound until optimal or budget.
  while (epsilon > 1.0 && expansions_ < max_iterations_) {
    epsilon = std::max(1.0, epsilon - decrement);
    for (const int s : incons_) {
      in_incons_[s] = 0;
      in_open_[s] = 1;
    }
    incons_.clear();
    std::fill(closed_.begin(), closed_.end(), 0);
    if (!improvePath(epsilon, cancel_checker)) {
      break;  // budget hit; keep the best path found so far
    }
  }

  if (g_[goal_idx_] == kInf) {
    throw nav2_core::NoValidPathCouldBeFound("ARA* found no path from start to goal");
  }

  // Reconstruct the (8-connected) cell chain from goal to start.
  std::vector<int> chain;
  for (int i = goal_idx_; i != start_idx_; i = parent_[i]) {
    chain.push_back(i);
    if (parent_[i] < 0) {
      throw nav2_core::NoValidPathCouldBeFound("ARA* path reconstruction broke");
    }
  }
  chain.push_back(start_idx_);
  std::reverse(chain.begin(), chain.end());

  std::vector<std::pair<double, double>> pts;
  pts.reserve(chain.size());
  for (const int idx : chain) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(
      static_cast<unsigned int>(idx % width_), static_cast<unsigned int>(idx / width_), wx, wy);
    pts.emplace_back(wx, wy);
  }
  pts.front() = {sx, sy};
  pts.back() = {gx, gy};

  plan.poses.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = pts[i].first;
    pose.pose.position.y = pts[i].second;
    if (i + 1 < pts.size()) {
      const double yaw = std::atan2(
        pts[i + 1].second - pts[i].second, pts[i + 1].first - pts[i].first);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    plan.poses.push_back(pose);
  }
  return plan;
}

}  // namespace nav2_ara_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_ara_star_planner::ARAStarPlanner, nav2_core::GlobalPlanner)
