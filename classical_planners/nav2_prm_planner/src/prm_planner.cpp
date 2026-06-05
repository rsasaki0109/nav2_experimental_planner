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

#include "nav2_prm_planner/prm_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_prm_planner
{

void PRMPlanner::configure(
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
  declare_parameter_if_not_declared(node, name_ + ".num_samples", rclcpp::ParameterValue(500));
  declare_parameter_if_not_declared(
    node, name_ + ".connection_radius", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name_ + ".max_neighbours", rclcpp::ParameterValue(10));
  declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name_ + ".random_seed", rclcpp::ParameterValue(1));

  node->get_parameter(name_ + ".num_samples", num_samples_);
  node->get_parameter(name_ + ".connection_radius", connection_radius_);
  node->get_parameter(name_ + ".max_neighbours", max_neighbours_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".random_seed", random_seed_);

  RCLCPP_INFO(
    logger_, "PRMPlanner '%s' configured: num_samples=%d radius=%.2f max_neighbours=%d",
    name_.c_str(), num_samples_, connection_radius_, max_neighbours_);
}

void PRMPlanner::cleanup() {}
void PRMPlanner::activate() {}
void PRMPlanner::deactivate() {}

bool PRMPlanner::pointFree(double wx, double wy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;
  }
  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return false;
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) {
    return false;
  }
  return true;
}

bool PRMPlanner::edgeFree(double ax, double ay, double bx, double by) const
{
  const double dist = std::hypot(bx - ax, by - ay);
  const double step = std::max(interpolation_resolution_, 1e-3);
  const int samples = std::max(1, static_cast<int>(std::ceil(dist / step)));
  for (int s = 0; s <= samples; ++s) {
    const double t = static_cast<double>(s) / static_cast<double>(samples);
    if (!pointFree(ax + t * (bx - ax), ay + t * (by - ay))) {
      return false;
    }
  }
  return true;
}

nav_msgs::msg::Path PRMPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  auto node = node_.lock();
  plan.header.stamp = node ? node->now() : rclcpp::Clock().now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerTFError(
            "PRMPlanner expects start/goal in the " + global_frame_ + " frame");
  }
  const double sx = start.pose.position.x;
  const double sy = start.pose.position.y;
  const double gx = goal.pose.position.x;
  const double gy = goal.pose.position.y;
  if (!pointFree(sx, sy)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal / out-of-bounds cell");
  }
  if (!pointFree(gx, gy)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal / out-of-bounds cell");
  }

  // Sampling bounds: the costmap extent in world coordinates.
  const double min_x = costmap_->getOriginX();
  const double min_y = costmap_->getOriginY();
  const double max_x = min_x + costmap_->getSizeInMetersX();
  const double max_y = min_y + costmap_->getSizeInMetersY();

  std::mt19937 rng(static_cast<std::mt19937::result_type>(random_seed_));
  std::uniform_real_distribution<double> ux(min_x, max_x);
  std::uniform_real_distribution<double> uy(min_y, max_y);

  // Milestones: index 0 = start, 1 = goal, then collision-free samples.
  std::vector<Milestone> nodes;
  nodes.push_back({sx, sy});
  nodes.push_back({gx, gy});

  const int max_attempts = std::max(1, num_samples_) * 30;
  int attempts = 0;
  while (static_cast<int>(nodes.size()) < num_samples_ + 2 && attempts < max_attempts) {
    ++attempts;
    if ((attempts & 0x3FF) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    const double rx = ux(rng);
    const double ry = uy(rng);
    if (pointFree(rx, ry)) {
      nodes.push_back({rx, ry});
    }
  }

  // Wire the roadmap: connect each milestone to its nearest neighbours within
  // connection_radius (capped at max_neighbours) if the straight edge is free.
  const int n = static_cast<int>(nodes.size());
  std::vector<std::vector<Edge>> adj(n);
  std::unordered_set<int64_t> wired;  // encoded (min,max) pairs already attempted
  std::vector<std::pair<double, int>> cand;
  cand.reserve(n);
  for (int i = 0; i < n; ++i) {
    if ((i & 0x3F) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    cand.clear();
    for (int j = 0; j < n; ++j) {
      if (j == i) {
        continue;
      }
      const double d = std::hypot(nodes[i].x - nodes[j].x, nodes[i].y - nodes[j].y);
      if (d <= connection_radius_) {
        cand.emplace_back(d, j);
      }
    }
    const int k = std::min(static_cast<int>(cand.size()), std::max(1, max_neighbours_));
    std::partial_sort(cand.begin(), cand.begin() + k, cand.end());
    for (int c = 0; c < k; ++c) {
      const int j = cand[c].second;
      const int64_t key = static_cast<int64_t>(std::min(i, j)) * n + std::max(i, j);
      if (wired.count(key)) {
        continue;
      }
      wired.insert(key);
      if (edgeFree(nodes[i].x, nodes[i].y, nodes[j].x, nodes[j].y)) {
        adj[i].push_back({j, cand[c].first});
        adj[j].push_back({i, cand[c].first});
      }
    }
  }

  // Dijkstra from start (0) to goal (1).
  std::vector<double> dist(n, std::numeric_limits<double>::max());
  std::vector<int> prev(n, -1);
  using QItem = std::pair<double, int>;  // (cost, node)
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
  dist[0] = 0.0;
  pq.push({0.0, 0});
  while (!pq.empty()) {
    const auto [d, u] = pq.top();
    pq.pop();
    if (d > dist[u]) {
      continue;
    }
    if (u == 1) {
      break;
    }
    for (const auto & e : adj[u]) {
      const double nd = d + e.weight;
      if (nd < dist[e.to]) {
        dist[e.to] = nd;
        prev[e.to] = u;
        pq.push({nd, e.to});
      }
    }
  }

  if (prev[1] < 0 && dist[1] == std::numeric_limits<double>::max()) {
    throw nav2_core::NoValidPathCouldBeFound(
            "PRM roadmap did not connect start and goal");
  }

  // Walk predecessors from the goal back to the start, then reverse.
  std::vector<int> path_idx;
  for (int i = 1; i != -1; i = prev[i]) {
    path_idx.push_back(i);
  }
  std::reverse(path_idx.begin(), path_idx.end());

  plan.poses.reserve(path_idx.size());
  for (std::size_t i = 0; i < path_idx.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = nodes[path_idx[i]].x;
    pose.pose.position.y = nodes[path_idx[i]].y;
    if (i + 1 < path_idx.size()) {
      const double yaw = std::atan2(
        nodes[path_idx[i + 1]].y - nodes[path_idx[i]].y,
        nodes[path_idx[i + 1]].x - nodes[path_idx[i]].x);
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

}  // namespace nav2_prm_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_prm_planner::PRMPlanner, nav2_core::GlobalPlanner)
