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

#include "nav2_visibility_graph_planner/visibility_graph_planner.hpp"

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

namespace nav2_visibility_graph_planner
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

void VisibilityGraphPlanner::configure(
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
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".max_corners", rclcpp::ParameterValue(1500));

  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".max_corners", max_corners_);

  RCLCPP_INFO(
    logger_, "VisibilityGraphPlanner '%s' configured: lethal_threshold=%d max_corners=%d",
    name_.c_str(), lethal_threshold_, max_corners_);
}

void VisibilityGraphPlanner::cleanup() {}
void VisibilityGraphPlanner::activate() {}
void VisibilityGraphPlanner::deactivate() {}

bool VisibilityGraphPlanner::isFree(int x, int y) const
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

bool VisibilityGraphPlanner::isConvexCorner(int x, int y) const
{
  if (!isFree(x, y)) {
    return false;
  }
  // A free cell is a convex obstacle corner if a diagonal neighbour is blocked
  // while the two orthogonal cells flanking that diagonal are free (so the cell
  // wraps around a protruding obstacle corner).
  for (int dy = -1; dy <= 1; dy += 2) {
    for (int dx = -1; dx <= 1; dx += 2) {
      if (!isFree(x + dx, y + dy) && isFree(x + dx, y) && isFree(x, y + dy)) {
        return true;
      }
    }
  }
  return false;
}

bool VisibilityGraphPlanner::lineOfSight(int ax, int ay, int bx, int by) const
{
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

nav_msgs::msg::Path VisibilityGraphPlanner::createPlan(
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
            "VisibilityGraphPlanner expects start/goal in the " + global_frame_ + " frame");
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

  // Graph vertices: start (0), goal (1), then obstacle convex corners.
  std::vector<int> node_x;
  std::vector<int> node_y;
  node_x.push_back(start_x);
  node_y.push_back(start_y);
  node_x.push_back(goal_x);
  node_y.push_back(goal_y);

  bool truncated = false;
  for (int y = 0; y < height_ && !truncated; ++y) {
    if ((y & 0x1F) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    for (int x = 0; x < width_; ++x) {
      if (isConvexCorner(x, y)) {
        node_x.push_back(x);
        node_y.push_back(y);
        if (static_cast<int>(node_x.size()) >= max_corners_ + 2) {
          truncated = true;
          break;
        }
      }
    }
  }
  if (truncated) {
    RCLCPP_WARN(
      logger_,
      "VisibilityGraphPlanner hit max_corners=%d; graph truncated, path may be suboptimal",
      max_corners_);
  }

  const int v = static_cast<int>(node_x.size());

  // World coordinates of every vertex (cell centres; ends pinned to start/goal).
  std::vector<double> wx(v);
  std::vector<double> wy(v);
  for (int i = 0; i < v; ++i) {
    costmap_->mapToWorld(
      static_cast<unsigned int>(node_x[i]), static_cast<unsigned int>(node_y[i]), wx[i], wy[i]);
  }
  wx[0] = sx;
  wy[0] = sy;
  wx[1] = gx;
  wy[1] = gy;

  // Build visibility edges (undirected) between mutually visible vertices.
  std::vector<std::vector<std::pair<int, double>>> adj(v);
  for (int i = 0; i < v; ++i) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    for (int j = i + 1; j < v; ++j) {
      if (lineOfSight(node_x[i], node_y[i], node_x[j], node_y[j])) {
        const double w = std::hypot(wx[i] - wx[j], wy[i] - wy[j]);
        adj[i].push_back({j, w});
        adj[j].push_back({i, w});
      }
    }
  }

  // A* over the visibility graph (Euclidean heuristic to the goal).
  std::vector<double> g(v, kInf);
  std::vector<int> parent(v, -1);
  std::vector<char> closed(v, 0);
  auto heur = [&](int i) {return std::hypot(wx[i] - gx, wy[i] - gy);};
  using QItem = std::pair<double, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
  g[0] = 0.0;
  open.push({heur(0), 0});
  bool found = false;
  while (!open.empty()) {
    const int u = open.top().second;
    open.pop();
    if (closed[u]) {
      continue;
    }
    closed[u] = 1;
    if (u == 1) {
      found = true;
      break;
    }
    for (const auto & e : adj[u]) {
      const double ng = g[u] + e.second;
      if (ng < g[e.first]) {
        g[e.first] = ng;
        parent[e.first] = u;
        open.push({ng + heur(e.first), e.first});
      }
    }
  }

  if (!found) {
    throw nav2_core::NoValidPathCouldBeFound(
            "Visibility graph did not connect start and goal");
  }

  // Reconstruct the vertex chain (goal -> start), reverse.
  std::vector<int> chain;
  for (int i = 1; i != -1; i = parent[i]) {
    chain.push_back(i);
  }
  std::reverse(chain.begin(), chain.end());

  // Densify each (line-of-sight-clear) straight segment for the controller.
  const double step = std::max(interpolation_resolution_, 1e-3);
  std::vector<std::pair<double, double>> pts;
  for (std::size_t s = 0; s + 1 < chain.size(); ++s) {
    const double ax = wx[chain[s]];
    const double ay = wy[chain[s]];
    const double bx = wx[chain[s + 1]];
    const double by = wy[chain[s + 1]];
    const double seg = std::hypot(bx - ax, by - ay);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / step)));
    for (int k = 0; k < n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      pts.emplace_back(ax + t * (bx - ax), ay + t * (by - ay));
    }
  }
  pts.emplace_back(gx, gy);
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

}  // namespace nav2_visibility_graph_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_visibility_graph_planner::VisibilityGraphPlanner, nav2_core::GlobalPlanner)
