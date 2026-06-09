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

#include "nav2_dstar_lite_planner/dstar_lite_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_dstar_lite_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309514880;
}  // namespace

bool DStarLitePlanner::keyLess(const Key & a, const Key & b)
{
  return a.k1 < b.k1 || (a.k1 == b.k1 && a.k2 < b.k2);
}

bool DStarLitePlanner::QGreater::operator()(const QEntry & a, const QEntry & b) const
{
  // priority_queue is a max-heap; invert so the smallest key sits on top.
  return keyLess(b.key, a.key);
}

void DStarLitePlanner::configure(
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
  declare_parameter_if_not_declared(node, name_ + ".cost_weight", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(
    node, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  node->get_parameter(name_ + ".cost_weight", cost_weight_);
  node->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(
    logger_, "DStarLitePlanner '%s' configured: cost_weight=%.2f heuristic_weight=%.2f",
    name_.c_str(), cost_weight_, heuristic_weight_);
}

void DStarLitePlanner::cleanup()
{
  initialized_ = false;
  g_.clear();
  rhs_.clear();
  snapshot_.clear();
  in_queue_.clear();
  open_ = decltype(open_)();
}
void DStarLitePlanner::activate() {}
void DStarLitePlanner::deactivate() {}

bool DStarLitePlanner::blocked(int idx) const
{
  const unsigned char cost = snapshot_[idx];
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return true;
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return !allow_unknown_;
  }
  return false;
}

double DStarLitePlanner::normCost(int idx) const
{
  const unsigned char cost = snapshot_[idx];
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return 0.0;
  }
  // Normalise 0..(INSCRIBED-1) into 0..1.
  const double denom = static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  return static_cast<double>(cost) / denom;
}

double DStarLitePlanner::edgeCost(int a, int b) const
{
  if (blocked(a) || blocked(b)) {
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

double DStarLitePlanner::heuristic(int a, int b) const
{
  const double dx = static_cast<double>(a % width_) - static_cast<double>(b % width_);
  const double dy = static_cast<double>(a / width_) - static_cast<double>(b / width_);
  return heuristic_weight_ * std::hypot(dx, dy);
}

std::vector<int> DStarLitePlanner::neighbours(int idx) const
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

DStarLitePlanner::Key DStarLitePlanner::calcKey(int idx) const
{
  const double m = std::min(g_[idx], rhs_[idx]);
  Key k;
  k.k1 = m + heuristic(start_idx_, idx) + km_;
  k.k2 = m;
  return k;
}

double DStarLitePlanner::recomputeRhs(int idx) const
{
  if (idx == goal_idx_) {
    return 0.0;
  }
  double best = kInf;
  for (const int n : neighbours(idx)) {
    const double c = edgeCost(idx, n);
    if (c == kInf) {
      continue;
    }
    best = std::min(best, c + g_[n]);
  }
  return best;
}

void DStarLitePlanner::insertOrUpdate(int idx, const Key & key)
{
  in_queue_[idx] = key;
  open_.push({key, idx});
}

void DStarLitePlanner::removeFromQueue(int idx)
{
  in_queue_.erase(idx);
}

bool DStarLitePlanner::topValid(Key & key, int & idx)
{
  while (!open_.empty()) {
    const QEntry top = open_.top();
    auto it = in_queue_.find(top.idx);
    if (it != in_queue_.end() && it->second.k1 == top.key.k1 && it->second.k2 == top.key.k2) {
      key = top.key;
      idx = top.idx;
      return true;
    }
    open_.pop();  // stale entry
  }
  return false;
}

void DStarLitePlanner::updateVertex(int idx)
{
  if (idx != goal_idx_) {
    rhs_[idx] = recomputeRhs(idx);
  }
  if (g_[idx] != rhs_[idx]) {
    insertOrUpdate(idx, calcKey(idx));
  } else {
    removeFromQueue(idx);
  }
}

bool DStarLitePlanner::computeShortestPath(const std::function<bool()> & cancel_checker)
{
  const int max_iter = 4 * width_ * height_ + 100;
  int iter = 0;
  Key ktop;
  int u = -1;
  while (topValid(ktop, u)) {
    const Key kstart = calcKey(start_idx_);
    const bool consistent_start = (rhs_[start_idx_] == g_[start_idx_]);
    if (!keyLess(ktop, kstart) && consistent_start) {
      break;
    }
    if (++iter > max_iter) {
      return false;
    }
    if ((iter & 0x3FF) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }

    // Re-read the validated top (topValid already filled u / ktop).
    const Key k_old = ktop;
    const Key k_new = calcKey(u);
    if (keyLess(k_old, k_new)) {
      insertOrUpdate(u, k_new);
    } else if (g_[u] > rhs_[u]) {
      // Over-consistent: lower g to rhs and relax predecessors.
      g_[u] = rhs_[u];
      removeFromQueue(u);
      for (const int s : neighbours(u)) {
        updateVertex(s);
      }
    } else {
      // Under-consistent: raise g and re-evaluate the vertex and predecessors.
      g_[u] = kInf;
      updateVertex(u);
      for (const int s : neighbours(u)) {
        updateVertex(s);
      }
    }
  }
  return true;
}

nav_msgs::msg::Path DStarLitePlanner::createPlan(
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
            "DStarLitePlanner expects start/goal in the " + global_frame_ + " frame");
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

  const int w = static_cast<int>(costmap_->getSizeInCellsX());
  const int h = static_cast<int>(costmap_->getSizeInCellsY());
  const double ox = costmap_->getOriginX();
  const double oy = costmap_->getOriginY();
  const double res = costmap_->getResolution();

  // Decide whether we can repair the cached search or must start fresh. The
  // goal-rooted D* Lite state is only reusable when the grid geometry and the
  // goal are unchanged.
  const int goal_idx = static_cast<int>(gmy) * w + static_cast<int>(gmx);
  const bool geometry_same = initialized_ && w == width_ && h == height_ &&
    ox == origin_x_ && oy == origin_y_ && res == resolution_;
  const bool reuse = geometry_same && goal_idx == goal_idx_;

  width_ = w;
  height_ = h;
  origin_x_ = ox;
  origin_y_ = oy;
  resolution_ = res;
  goal_idx_ = goal_idx;
  start_idx_ = static_cast<int>(smy) * w + static_cast<int>(smx);

  const std::size_t cells = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

  if (!reuse) {
    // Fresh initialisation: clear g/rhs, seed the goal, reset km.
    g_.assign(cells, kInf);
    rhs_.assign(cells, kInf);
    snapshot_.assign(cells, 0);
    for (std::size_t i = 0; i < cells; ++i) {
      snapshot_[i] = costmap_->getCost(static_cast<unsigned int>(i % w),
          static_cast<unsigned int>(i / w));
    }
    in_queue_.clear();
    open_ = decltype(open_)();
    km_ = 0.0;
    last_start_idx_ = start_idx_;
    rhs_[goal_idx_] = 0.0;
    insertOrUpdate(goal_idx_, calcKey(goal_idx_));
    initialized_ = true;
  } else {
    // Incremental repair: shift the priority keys for the moved start, then
    // update only the vertices whose costs changed since the last plan.
    km_ += heuristic(last_start_idx_, start_idx_);
    last_start_idx_ = start_idx_;

    std::vector<int> dirty;
    for (std::size_t i = 0; i < cells; ++i) {
      const unsigned char now = costmap_->getCost(static_cast<unsigned int>(i % w),
          static_cast<unsigned int>(i / w));
      if (now != snapshot_[i]) {
        snapshot_[i] = now;
        dirty.push_back(static_cast<int>(i));
      }
    }
    // A changed cell affects its own rhs and that of every neighbour (edges are
    // symmetric), so re-evaluate the cell plus its 8-neighbourhood.
    for (const int c : dirty) {
      updateVertex(c);
      for (const int n : neighbours(c)) {
        updateVertex(n);
      }
    }
  }

  if (blocked(start_idx_)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal cell");
  }
  if (blocked(goal_idx_)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal cell");
  }

  if (cancel_checker && cancel_checker()) {
    throw nav2_core::PlannerCancelled("Planning cancelled");
  }

  if (!computeShortestPath(cancel_checker)) {
    throw nav2_core::NoValidPathCouldBeFound("D* Lite exceeded its expansion budget");
  }

  if (g_[start_idx_] == kInf) {
    throw nav2_core::NoValidPathCouldBeFound("D* Lite found no path from start to goal");
  }

  // Extract the path by greedily following minimum-cost successors from start.
  std::vector<int> cell_path;
  cell_path.push_back(start_idx_);
  int cur = start_idx_;
  const int max_steps = w * h;
  int steps = 0;
  while (cur != goal_idx_ && steps++ < max_steps) {
    double best = kInf;
    int next = -1;
    for (const int n : neighbours(cur)) {
      const double c = edgeCost(cur, n);
      if (c == kInf) {
        continue;
      }
      const double val = c + g_[n];
      if (val < best) {
        best = val;
        next = n;
      }
    }
    if (next < 0 || best == kInf) {
      throw nav2_core::NoValidPathCouldBeFound("D* Lite path extraction stalled");
    }
    cur = next;
    cell_path.push_back(cur);
  }
  if (cur != goal_idx_) {
    throw nav2_core::NoValidPathCouldBeFound("D* Lite path extraction did not reach the goal");
  }

  // Convert cells to world poses; pin the exact start/goal positions at the ends.
  std::vector<std::pair<double, double>> pts;
  pts.reserve(cell_path.size());
  for (const int idx : cell_path) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(
      static_cast<unsigned int>(idx % w), static_cast<unsigned int>(idx / w), wx, wy);
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

}  // namespace nav2_dstar_lite_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dstar_lite_planner::DStarLitePlanner, nav2_core::GlobalPlanner)
