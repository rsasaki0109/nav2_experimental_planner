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

#include "nav2_rrt_planner/rrt_star_planner.hpp"

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

namespace nav2_rrt_planner
{

void RRTStarPlanner::configure(
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
  declare_parameter_if_not_declared(node, name_ + ".max_iterations", rclcpp::ParameterValue(4000));
  declare_parameter_if_not_declared(node, name_ + ".step_size", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".goal_bias", rclcpp::ParameterValue(0.10));
  declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".rewire_radius", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name_ + ".random_seed", rclcpp::ParameterValue(1));

  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".step_size", step_size_);
  node->get_parameter(name_ + ".goal_bias", goal_bias_);
  node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
  node->get_parameter(name_ + ".rewire_radius", rewire_radius_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".random_seed", random_seed_);

  RCLCPP_INFO(
    logger_, "RRTStarPlanner '%s' configured: max_iter=%d step=%.2f goal_bias=%.2f",
    name_.c_str(), max_iterations_, step_size_, goal_bias_);
}

void RRTStarPlanner::cleanup() {}
void RRTStarPlanner::activate() {}
void RRTStarPlanner::deactivate() {}

bool RRTStarPlanner::pointFree(double wx, double wy) const
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

bool RRTStarPlanner::edgeFree(double ax, double ay, double bx, double by) const
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

nav_msgs::msg::Path RRTStarPlanner::createPlan(
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
            "RRTStarPlanner expects start/goal in the " + global_frame_ + " frame");
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
  std::uniform_real_distribution<double> up(0.0, 1.0);

  std::vector<Node> tree;
  tree.push_back({sx, sy, -1, 0.0});

  int goal_node = -1;
  double goal_cost = std::numeric_limits<double>::max();

  for (int it = 0; it < max_iterations_; ++it) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }

    // Sample (goal-biased).
    double rx;
    double ry;
    if (up(rng) < goal_bias_) {
      rx = gx;
      ry = gy;
    } else {
      rx = ux(rng);
      ry = uy(rng);
    }

    // Nearest existing node.
    int nearest = 0;
    double nearest_d = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d = std::hypot(rx - tree[i].x, ry - tree[i].y);
      if (d < nearest_d) {
        nearest_d = d;
        nearest = static_cast<int>(i);
      }
    }

    // Steer from nearest toward the sample by at most step_size_.
    double nx;
    double ny;
    if (nearest_d <= step_size_) {
      nx = rx;
      ny = ry;
    } else {
      const double s = step_size_ / nearest_d;
      nx = tree[nearest].x + s * (rx - tree[nearest].x);
      ny = tree[nearest].y + s * (ry - tree[nearest].y);
    }
    if (!pointFree(nx, ny) || !edgeFree(tree[nearest].x, tree[nearest].y, nx, ny)) {
      continue;
    }

    // Choose the best parent within the rewire radius (RRT* connect step).
    int best_parent = nearest;
    double best_cost = tree[nearest].cost + std::hypot(nx - tree[nearest].x, ny - tree[nearest].y);
    std::vector<int> near_ids;
    for (std::size_t i = 0; i < tree.size(); ++i) {
      const double d = std::hypot(nx - tree[i].x, ny - tree[i].y);
      if (d <= rewire_radius_) {
        near_ids.push_back(static_cast<int>(i));
        const double c = tree[i].cost + d;
        if (c < best_cost && edgeFree(tree[i].x, tree[i].y, nx, ny)) {
          best_cost = c;
          best_parent = static_cast<int>(i);
        }
      }
    }

    const int new_id = static_cast<int>(tree.size());
    tree.push_back({nx, ny, best_parent, best_cost});

    // Rewire neighbours through the new node if that lowers their cost.
    for (const int i : near_ids) {
      const double d = std::hypot(nx - tree[i].x, ny - tree[i].y);
      const double c = best_cost + d;
      if (c < tree[i].cost && edgeFree(nx, ny, tree[i].x, tree[i].y)) {
        tree[i].parent = new_id;
        tree[i].cost = c;
      }
    }

    // Track the best node that reaches the goal region.
    const double dg = std::hypot(nx - gx, ny - gy);
    if (dg <= goal_tolerance_ && edgeFree(nx, ny, gx, gy)) {
      const double total = best_cost + dg;
      if (total < goal_cost) {
        goal_cost = total;
        goal_node = new_id;
      }
    }
  }

  if (goal_node < 0) {
    throw nav2_core::NoValidPathCouldBeFound(
            "RRT* found no collision-free path within max_iterations");
  }

  // Walk parents from the goal node back to the start, then reverse.
  std::vector<std::pair<double, double>> pts;
  pts.emplace_back(gx, gy);
  for (int i = goal_node; i != -1; i = tree[i].parent) {
    pts.emplace_back(tree[i].x, tree[i].y);
  }
  std::reverse(pts.begin(), pts.end());

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

}  // namespace nav2_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRTStarPlanner, nav2_core::GlobalPlanner)
