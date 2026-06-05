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

#include "nav2_rrt_planner/rrt_connect_planner.hpp"

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

void RRTConnectPlanner::configure(
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
  declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name_ + ".random_seed", rclcpp::ParameterValue(1));

  node->get_parameter(name_ + ".max_iterations", max_iterations_);
  node->get_parameter(name_ + ".step_size", step_size_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".random_seed", random_seed_);

  RCLCPP_INFO(
    logger_, "RRTConnectPlanner '%s' configured: max_iter=%d step=%.2f",
    name_.c_str(), max_iterations_, step_size_);
}

void RRTConnectPlanner::cleanup() {}
void RRTConnectPlanner::activate() {}
void RRTConnectPlanner::deactivate() {}

bool RRTConnectPlanner::pointFree(double wx, double wy) const
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

bool RRTConnectPlanner::edgeFree(double ax, double ay, double bx, double by) const
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

RRTConnectPlanner::Status RRTConnectPlanner::extend(
  std::vector<Node> & tree, double tx, double ty, int & out_id) const
{
  // Nearest existing node to the target.
  int nearest = 0;
  double nearest_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < tree.size(); ++i) {
    const double d = std::hypot(tx - tree[i].x, ty - tree[i].y);
    if (d < nearest_d) {
      nearest_d = d;
      nearest = static_cast<int>(i);
    }
  }

  // Steer from the nearest node toward the target by at most step_size_.
  bool reached;
  double nx;
  double ny;
  if (nearest_d <= step_size_) {
    nx = tx;
    ny = ty;
    reached = true;
  } else {
    const double s = step_size_ / nearest_d;
    nx = tree[nearest].x + s * (tx - tree[nearest].x);
    ny = tree[nearest].y + s * (ty - tree[nearest].y);
    reached = false;
  }

  if (!pointFree(nx, ny) || !edgeFree(tree[nearest].x, tree[nearest].y, nx, ny)) {
    return Status::kTrapped;
  }

  out_id = static_cast<int>(tree.size());
  tree.push_back({nx, ny, nearest});
  return reached ? Status::kReached : Status::kAdvanced;
}

RRTConnectPlanner::Status RRTConnectPlanner::connect(
  std::vector<Node> & tree, double tx, double ty, int & out_id) const
{
  Status s = Status::kAdvanced;
  while (s == Status::kAdvanced) {
    s = extend(tree, tx, ty, out_id);
  }
  return s;  // kReached or kTrapped
}

nav_msgs::msg::Path RRTConnectPlanner::createPlan(
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
            "RRTConnectPlanner expects start/goal in the " + global_frame_ + " frame");
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

  std::vector<Node> tree_start;
  std::vector<Node> tree_goal;
  tree_start.push_back({sx, sy, -1});
  tree_goal.push_back({gx, gy, -1});

  // `tree_a` is extended toward the random sample; `tree_b` then tries to
  // connect to the new node. The two are swapped each iteration so growth
  // alternates between the start-rooted and goal-rooted trees.
  std::vector<Node> * tree_a = &tree_start;
  std::vector<Node> * tree_b = &tree_goal;
  bool a_is_start = true;

  for (int it = 0; it < max_iterations_; ++it) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }

    const double rx = ux(rng);
    const double ry = uy(rng);

    int a_id = -1;
    if (extend(*tree_a, rx, ry, a_id) != Status::kTrapped) {
      const double cx = (*tree_a)[a_id].x;
      const double cy = (*tree_a)[a_id].y;
      int b_id = -1;
      if (connect(*tree_b, cx, cy, b_id) == Status::kReached) {
        // Trees met. Walk each branch back to its root.
        std::vector<std::pair<double, double>> branch_a;
        for (int i = a_id; i != -1; i = (*tree_a)[i].parent) {
          branch_a.emplace_back((*tree_a)[i].x, (*tree_a)[i].y);
        }
        std::vector<std::pair<double, double>> branch_b;
        for (int i = b_id; i != -1; i = (*tree_b)[i].parent) {
          branch_b.emplace_back((*tree_b)[i].x, (*tree_b)[i].y);
        }

        // Orient both branches so the path runs start -> goal. branch_* run
        // from the connection node back to their roots; the start branch must
        // read root..node, the goal branch node..root (skipping the shared
        // connection node to avoid duplication).
        std::vector<std::pair<double, double>> start_branch =
          a_is_start ? branch_a : branch_b;   // connection -> start root
        std::vector<std::pair<double, double>> goal_branch =
          a_is_start ? branch_b : branch_a;   // connection -> goal root
        std::reverse(start_branch.begin(), start_branch.end());  // start root -> connection

        std::vector<std::pair<double, double>> pts = std::move(start_branch);
        for (std::size_t i = 1; i < goal_branch.size(); ++i) {
          pts.push_back(goal_branch[i]);
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
    }

    std::swap(tree_a, tree_b);
    a_is_start = !a_is_start;
  }

  throw nav2_core::NoValidPathCouldBeFound(
          "RRT-Connect found no collision-free path within max_iterations");
}

}  // namespace nav2_rrt_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRTConnectPlanner, nav2_core::GlobalPlanner)
