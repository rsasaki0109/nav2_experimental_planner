// Copyright 2026 nav2_diffusion_planner contributors
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

#include "nav2_diffusion_global_planner/diffusion_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_diffusion_global_planner
{

void DiffusionGlobalPlanner::configure(
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
  declare_parameter_if_not_declared(node, name_ + ".num_candidates", rclcpp::ParameterValue(11));
  declare_parameter_if_not_declared(node, name_ + ".num_points", rclcpp::ParameterValue(40));
  declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name_ + ".max_bow_fraction", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, name_ + ".model_plugin", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, name_ + ".model_path", rclcpp::ParameterValue(std::string("")));

  node->get_parameter(name_ + ".num_candidates", num_candidates_);
  node->get_parameter(name_ + ".num_points", num_points_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".max_bow_fraction", max_bow_fraction_);
  node->get_parameter(name_ + ".model_plugin", model_plugin_);
  node->get_parameter(name_ + ".model_path", model_path_);

  if (model_plugin_.empty()) {
    model_ = std::make_shared<nav2_diffusion_core::FanPathModel>(max_bow_fraction_);
  } else {
    model_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_diffusion_core::PathModel>>(
      "nav2_diffusion_core", "nav2_diffusion_core::PathModel");
    model_ = model_loader_->createSharedInstance(model_plugin_);
    model_->configure(model_path_);
  }

  RCLCPP_INFO(
    logger_, "DiffusionGlobalPlanner '%s' configured: model='%s', %d candidates, %d points",
    name_.c_str(), model_->name().c_str(), num_candidates_, num_points_);
}

void DiffusionGlobalPlanner::cleanup()
{
  model_.reset();
  model_loader_.reset();
}

void DiffusionGlobalPlanner::activate() {}

void DiffusionGlobalPlanner::deactivate() {}

bool DiffusionGlobalPlanner::isCellTraversable(double wx, double wy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;  // outside the costmap bounds
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

bool DiffusionGlobalPlanner::isPathValid(const nav2_diffusion_core::PathCandidate & path) const
{
  const double step = std::max(interpolation_resolution_, 1e-3);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto & a = path.points[i - 1];
    const auto & b = path.points[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    const int samples = std::max(1, static_cast<int>(std::ceil(seg / step)));
    for (int s = 0; s <= samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(samples);
      if (!isCellTraversable(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))) {
        return false;
      }
    }
  }
  return true;
}

nav_msgs::msg::Path DiffusionGlobalPlanner::createPlan(
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
            "DiffusionGlobalPlanner expects start/goal in the " + global_frame_ + " frame");
  }
  if (!isCellTraversable(start.pose.position.x, start.pose.position.y)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal / out-of-bounds cell");
  }
  if (!isCellTraversable(goal.pose.position.x, goal.pose.position.y)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal / out-of-bounds cell");
  }

  // Propose: ask the generative model for K candidate paths.
  nav2_diffusion_core::PathContext ctx;
  ctx.start_x = start.pose.position.x;
  ctx.start_y = start.pose.position.y;
  ctx.goal_x = goal.pose.position.x;
  ctx.goal_y = goal.pose.position.y;
  ctx.num_candidates = num_candidates_;
  ctx.num_points = num_points_;
  const auto candidates = model_->generate(ctx);

  // Dispose + select: keep the shortest collision-free candidate.
  const nav2_diffusion_core::PathCandidate * best = nullptr;
  double best_length = std::numeric_limits<double>::max();
  for (const auto & candidate : candidates) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    if (candidate.size() < 2 || !isPathValid(candidate)) {
      continue;
    }
    const double length = nav2_diffusion_core::pathLength(candidate);
    if (length < best_length) {
      best_length = length;
      best = &candidate;
    }
  }

  if (best == nullptr) {
    throw nav2_core::NoValidPathCouldBeFound(
            "No collision-free candidate path among the generated proposals");
  }

  // Build the nav_msgs::Path, orienting each pose toward the next waypoint.
  plan.poses.reserve(best->points.size());
  for (std::size_t i = 0; i < best->points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = best->points[i].x;
    pose.pose.position.y = best->points[i].y;
    double yaw;
    if (i + 1 < best->points.size()) {
      yaw = std::atan2(
        best->points[i + 1].y - best->points[i].y,
        best->points[i + 1].x - best->points[i].x);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      pose.pose.orientation = goal.pose.orientation;  // final pose keeps goal heading
    }
    plan.poses.push_back(pose);
  }

  return plan;
}

}  // namespace nav2_diffusion_global_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_diffusion_global_planner::DiffusionGlobalPlanner, nav2_core::GlobalPlanner)
