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

#include "nav2_lazy_theta_star_planner/lazy_theta_star_planner.hpp"

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

namespace nav2_lazy_theta_star_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

void LazyThetaStarPlanner::configure(
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
  declare_parameter_if_not_declared(
    node, name_ + ".heuristic_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));

  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".heuristic_weight", heuristic_weight_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  RCLCPP_INFO(
    logger_, "LazyThetaStarPlanner '%s' configured: lethal_threshold=%d allow_unknown=%d",
    name_.c_str(), lethal_threshold_, allow_unknown_ ? 1 : 0);
}

void LazyThetaStarPlanner::cleanup() {}
void LazyThetaStarPlanner::activate() {}
void LazyThetaStarPlanner::deactivate() {}

bool LazyThetaStarPlanner::isFree(int x, int y) const
{
  if (x < 0 || y < 0 || x >= width_ || y >= height_) {
    return false;
  }
  const unsigned char cost = costmap_->getCost(
    static_cast<unsigned int>(x), static_cast<unsigned int>(y));
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_threshold_;
}

double LazyThetaStarPlanner::dist(int ax, int ay, int bx, int by) const
{
  return std::hypot(static_cast<double>(ax - bx), static_cast<double>(ay - by));
}

bool LazyThetaStarPlanner::lineOfSight(int ax, int ay, int bx, int by) const
{
  // Sample the straight segment between the two cell centres at the same
  // resolution the final path is densified at, rejecting if any sampled cell is
  // an obstacle. Using the densification sampling here keeps line-of-sight and
  // the emitted path consistent: every segment accepted by the search produces a
  // collision-free densified path.
  double awx = 0.0;
  double awy = 0.0;
  double bwx = 0.0;
  double bwy = 0.0;
  costmap_->mapToWorld(
    static_cast<unsigned int>(ax), static_cast<unsigned int>(ay), awx, awy);
  costmap_->mapToWorld(
    static_cast<unsigned int>(bx), static_cast<unsigned int>(by), bwx, bwy);

  const double seg = std::hypot(bwx - awx, bwy - awy);
  const double step = std::max(interpolation_resolution_, 1e-3);
  const int n = std::max(1, static_cast<int>(std::ceil(seg / step)));
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    const double wx = awx + t * (bwx - awx);
    const double wy = awy + t * (bwy - awy);
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return false;
    }
    if (!isFree(static_cast<int>(mx), static_cast<int>(my))) {
      return false;
    }
  }
  return true;
}

std::vector<int> LazyThetaStarPlanner::neighbours(int idx) const
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
      if (isFree(nx, ny)) {
        out.push_back(index(nx, ny));
      }
    }
  }
  return out;
}

nav_msgs::msg::Path LazyThetaStarPlanner::createPlan(
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
            "LazyThetaStarPlanner expects start/goal in the " + global_frame_ + " frame");
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
  const int start_x = static_cast<int>(smx);
  const int start_y = static_cast<int>(smy);
  const int goal_x = static_cast<int>(gmx);
  const int goal_y = static_cast<int>(gmy);

  if (!isFree(start_x, start_y)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal cell");
  }
  if (!isFree(goal_x, goal_y)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal cell");
  }

  if (cancel_checker && cancel_checker()) {
    throw nav2_core::PlannerCancelled("Planning cancelled");
  }

  const int start_idx = index(start_x, start_y);
  const int goal_idx = index(goal_x, goal_y);

  const std::size_t cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  std::vector<double> g(cells, kInf);
  std::vector<int> parent(cells, -1);
  std::vector<char> closed(cells, 0);
  using QItem = std::pair<double, int>;  // (f, idx)
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

  g[start_idx] = 0.0;
  parent[start_idx] = start_idx;
  open.push({heuristic_weight_ * dist(start_x, start_y, goal_x, goal_y), start_idx});

  int expansions = 0;
  const int max_expansions = static_cast<int>(cells) + 10;
  bool found = false;
  while (!open.empty()) {
    const int u = open.top().second;
    open.pop();
    if (closed[u]) {
      continue;
    }

    const int ux = u % width_;
    const int uy = u / width_;

    // SetVertex (lazy verification): if the optimistic grandparent link is not
    // actually visible, fall back to the best visible expanded neighbour.
    if (u != start_idx) {
      const int p = parent[u];
      if (p < 0 || !lineOfSight(p % width_, p / width_, ux, uy)) {
        double best = kInf;
        int best_parent = -1;
        for (const int n : neighbours(u)) {
          if (closed[n]) {
            const double c = g[n] + dist(n % width_, n / width_, ux, uy);
            if (c < best) {
              best = c;
              best_parent = n;
            }
          }
        }
        if (best_parent < 0) {
          continue;  // no valid visible parent yet; drop this stale entry
        }
        g[u] = best;
        parent[u] = best_parent;
      }
    }

    if (u == goal_idx) {
      found = true;
      break;
    }
    closed[u] = 1;
    if (++expansions > max_expansions) {
      break;
    }
    if ((expansions & 0x3FF) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }

    // Lazy successor generation: optimistically route each open neighbour
    // through u's parent (path 2), deferring the line-of-sight check.
    const int pu = parent[u];
    const int pux = pu % width_;
    const int puy = pu / width_;
    for (const int n : neighbours(u)) {
      if (closed[n]) {
        continue;
      }
      const double newg = g[pu] + dist(pux, puy, n % width_, n / width_);
      if (newg < g[n]) {
        g[n] = newg;
        parent[n] = pu;
        open.push({newg + heuristic_weight_ * dist(n % width_, n / width_, goal_x, goal_y), n});
      }
    }
  }

  if (!found) {
    throw nav2_core::NoValidPathCouldBeFound("Lazy Theta* found no path from start to goal");
  }

  // Reconstruct the any-angle vertex chain (goal -> start via parent), reverse.
  std::vector<int> vtx;
  for (int i = goal_idx; ; i = parent[i]) {
    vtx.push_back(i);
    if (i == start_idx) {
      break;
    }
  }
  std::reverse(vtx.begin(), vtx.end());

  // Vertex cells -> world centres.
  std::vector<std::pair<double, double>> verts;
  verts.reserve(vtx.size());
  for (const int idx : vtx) {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(
      static_cast<unsigned int>(idx % width_), static_cast<unsigned int>(idx / width_), wx, wy);
    verts.emplace_back(wx, wy);
  }
  verts.front() = {sx, sy};
  verts.back() = {gx, gy};

  // Densify each (line-of-sight-clear) straight segment for the controller.
  const double step = std::max(interpolation_resolution_, 1e-3);
  std::vector<std::pair<double, double>> pts;
  for (std::size_t s = 0; s + 1 < verts.size(); ++s) {
    const double ax = verts[s].first;
    const double ay = verts[s].second;
    const double bx = verts[s + 1].first;
    const double by = verts[s + 1].second;
    const double seg = std::hypot(bx - ax, by - ay);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / step)));
    for (int k = 0; k < n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      pts.emplace_back(ax + t * (bx - ax), ay + t * (by - ay));
    }
  }
  pts.push_back(verts.back());
  if (pts.size() < 2) {
    pts = {{sx, sy}, {gx, gy}};
  }

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

}  // namespace nav2_lazy_theta_star_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_lazy_theta_star_planner::LazyThetaStarPlanner, nav2_core::GlobalPlanner)
