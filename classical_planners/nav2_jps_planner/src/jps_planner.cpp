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

#include "nav2_jps_planner/jps_planner.hpp"

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

namespace nav2_jps_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kSqrt2 = 1.41421356237309514880;

int sgn(int v)
{
  return (v > 0) - (v < 0);
}
}  // namespace

void JPSPlanner::configure(
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

  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(
    logger_, "JPSPlanner '%s' configured: lethal_threshold=%d allow_unknown=%d",
    name_.c_str(), lethal_threshold_, allow_unknown_ ? 1 : 0);
}

void JPSPlanner::cleanup() {}
void JPSPlanner::activate() {}
void JPSPlanner::deactivate() {}

bool JPSPlanner::isFree(int x, int y) const
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

double JPSPlanner::octile(int ax, int ay, int bx, int by) const
{
  const int dx = std::abs(ax - bx);
  const int dy = std::abs(ay - by);
  const int dmin = std::min(dx, dy);
  const int dmax = std::max(dx, dy);
  return (dmax - dmin) + kSqrt2 * dmin;
}

int JPSPlanner::jump(int x, int y, int dx, int dy) const
{
  while (true) {
    x += dx;
    y += dy;
    if (!isFree(x, y)) {
      return -1;
    }
    if (x == gx_ && y == gy_) {
      return index(x, y);
    }
    if (dx != 0 && dy != 0) {
      // Diagonal: a forced neighbour appears when an orthogonal cell is blocked
      // but the cell beyond it (along the diagonal) is free.
      if ((isFree(x - dx, y + dy) && !isFree(x - dx, y)) ||
        (isFree(x + dx, y - dy) && !isFree(x, y - dy)))
      {
        return index(x, y);
      }
      // Expand the two straight components before stepping diagonally again.
      if (jump(x, y, dx, 0) != -1 || jump(x, y, 0, dy) != -1) {
        return index(x, y);
      }
    } else if (dx != 0) {
      // Horizontal straight move.
      if ((isFree(x + dx, y + 1) && !isFree(x, y + 1)) ||
        (isFree(x + dx, y - 1) && !isFree(x, y - 1)))
      {
        return index(x, y);
      }
    } else {
      // Vertical straight move.
      if ((isFree(x + 1, y + dy) && !isFree(x + 1, y)) ||
        (isFree(x - 1, y + dy) && !isFree(x - 1, y)))
      {
        return index(x, y);
      }
    }
  }
}

std::vector<std::pair<int, int>> JPSPlanner::prunedDirections(
  int x, int y, int pdx, int pdy) const
{
  std::vector<std::pair<int, int>> dirs;
  if (pdx == 0 && pdy == 0) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx != 0 || dy != 0) {
          dirs.emplace_back(dx, dy);
        }
      }
    }
    return dirs;
  }

  if (pdx != 0 && pdy != 0) {
    // Diagonal: natural neighbours are the diagonal and its two components.
    dirs.emplace_back(pdx, pdy);
    dirs.emplace_back(pdx, 0);
    dirs.emplace_back(0, pdy);
    if (!isFree(x - pdx, y) && isFree(x - pdx, y + pdy)) {
      dirs.emplace_back(-pdx, pdy);
    }
    if (!isFree(x, y - pdy) && isFree(x + pdx, y - pdy)) {
      dirs.emplace_back(pdx, -pdy);
    }
  } else if (pdx != 0) {
    // Horizontal: natural neighbour is straight ahead; forced ones appear when
    // a vertically-adjacent cell is blocked.
    dirs.emplace_back(pdx, 0);
    if (!isFree(x, y + 1) && isFree(x + pdx, y + 1)) {
      dirs.emplace_back(pdx, 1);
    }
    if (!isFree(x, y - 1) && isFree(x + pdx, y - 1)) {
      dirs.emplace_back(pdx, -1);
    }
  } else {
    // Vertical.
    dirs.emplace_back(0, pdy);
    if (!isFree(x + 1, y) && isFree(x + 1, y + pdy)) {
      dirs.emplace_back(1, pdy);
    }
    if (!isFree(x - 1, y) && isFree(x - 1, y + pdy)) {
      dirs.emplace_back(-1, pdy);
    }
  }
  return dirs;
}

nav_msgs::msg::Path JPSPlanner::createPlan(
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
            "JPSPlanner expects start/goal in the " + global_frame_ + " frame");
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
  gx_ = static_cast<int>(gmx);
  gy_ = static_cast<int>(gmy);

  if (!isFree(start_x, start_y)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal cell");
  }
  if (!isFree(gx_, gy_)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal cell");
  }

  if (cancel_checker && cancel_checker()) {
    throw nav2_core::PlannerCancelled("Planning cancelled");
  }

  const int start_idx = index(start_x, start_y);
  const int goal_idx = index(gx_, gy_);

  // A* over jump points.
  const std::size_t cells = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  std::vector<double> g(cells, kInf);
  std::vector<int> parent(cells, -1);
  std::vector<char> closed(cells, 0);
  using QItem = std::pair<double, int>;  // (f, idx)
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

  g[start_idx] = 0.0;
  open.push({octile(start_x, start_y, gx_, gy_), start_idx});

  int expansions = 0;
  const int max_expansions = static_cast<int>(cells) + 10;
  bool found = false;
  while (!open.empty()) {
    const int u = open.top().second;
    open.pop();
    if (closed[u]) {
      continue;
    }
    closed[u] = 1;
    if (u == goal_idx) {
      found = true;
      break;
    }
    if (++expansions > max_expansions) {
      break;
    }
    if ((expansions & 0x3FF) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }

    const int ux = u % width_;
    const int uy = u / width_;
    int pdx = 0;
    int pdy = 0;
    if (parent[u] != -1) {
      pdx = sgn(ux - parent[u] % width_);
      pdy = sgn(uy - parent[u] / width_);
    }

    for (const auto & d : prunedDirections(ux, uy, pdx, pdy)) {
      const int jp = jump(ux, uy, d.first, d.second);
      if (jp < 0 || closed[jp]) {
        continue;
      }
      const int jx = jp % width_;
      const int jy = jp / width_;
      const double ng = g[u] + octile(ux, uy, jx, jy);
      if (ng < g[jp]) {
        g[jp] = ng;
        parent[jp] = u;
        open.push({ng + octile(jx, jy, gx_, gy_), jp});
      }
    }
  }

  if (!found) {
    throw nav2_core::NoValidPathCouldBeFound("JPS found no path from start to goal");
  }

  // Reconstruct the sparse jump-point chain, then densify each straight/diagonal
  // segment cell by cell (JPS guarantees those segments are collision-free).
  std::vector<int> jp_chain;
  for (int i = goal_idx; i != -1; i = parent[i]) {
    jp_chain.push_back(i);
  }
  std::reverse(jp_chain.begin(), jp_chain.end());

  std::vector<std::pair<double, double>> pts;
  for (std::size_t s = 0; s + 1 < jp_chain.size(); ++s) {
    int ax = jp_chain[s] % width_;
    int ay = jp_chain[s] / width_;
    const int bx = jp_chain[s + 1] % width_;
    const int by = jp_chain[s + 1] / width_;
    const int dx = sgn(bx - ax);
    const int dy = sgn(by - ay);
    while (ax != bx || ay != by) {
      double wx = 0.0;
      double wy = 0.0;
      costmap_->mapToWorld(
        static_cast<unsigned int>(ax), static_cast<unsigned int>(ay), wx, wy);
      pts.emplace_back(wx, wy);
      ax += dx;
      ay += dy;
    }
  }
  // Append the goal cell centre (the loop above stops just before each segment end).
  {
    double wx = 0.0;
    double wy = 0.0;
    costmap_->mapToWorld(
      static_cast<unsigned int>(gx_), static_cast<unsigned int>(gy_), wx, wy);
    pts.emplace_back(wx, wy);
  }

  // Pin the exact start/goal world positions at the ends.
  if (pts.size() < 2) {
    pts = {{sx, sy}, {gx, gy}};
  } else {
    pts.front() = {sx, sy};
    pts.back() = {gx, gy};
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

}  // namespace nav2_jps_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_jps_planner::JPSPlanner, nav2_core::GlobalPlanner)
