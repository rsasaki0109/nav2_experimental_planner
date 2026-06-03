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

#include "nav2_diffusion_controller/diffusion_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_diffusion_controller
{

void DiffusionController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  base_frame_ = costmap_ros_->getBaseFrameID();

  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".desired_linear_speed", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".horizon", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".time_step", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".consider_unknown_lethal", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".desired_linear_speed", desired_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(plugin_name_ + ".horizon", horizon_);
  node->get_parameter(plugin_name_ + ".time_step", time_step_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".consider_unknown_lethal", consider_unknown_lethal_);

  kinematic_filter_ = std::make_shared<nav2_diffusion_safety::KinematicLimitsFilter>(
    max_linear_speed_, max_angular_speed_);
  footprint_filter_ = std::make_shared<nav2_diffusion_safety::FootprintCollisionFilter>(
    costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
    253.0, consider_unknown_lethal_);

  candidates_pub_ = node->create_publisher<nav2_diffusion_msgs::msg::TrajectoryCandidates>(
    plugin_name_ + "/trajectory_candidates", 1);
  safety_pub_ = node->create_publisher<nav2_diffusion_msgs::msg::SafetyState>(
    plugin_name_ + "/safety_state", 1);

  RCLCPP_INFO(
    logger_, "Configured DiffusionController '%s' (v0.1 skeleton)", plugin_name_.c_str());
}

void DiffusionController::cleanup()
{
  candidates_pub_.reset();
  safety_pub_.reset();
  kinematic_filter_.reset();
  footprint_filter_.reset();
}

void DiffusionController::activate()
{
  candidates_pub_->on_activate();
  safety_pub_->on_activate();
}

void DiffusionController::deactivate()
{
  candidates_pub_->on_deactivate();
  safety_pub_->on_deactivate();
}

void DiffusionController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void DiffusionController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limit_scale_ = 1.0;
    return;
  }
  if (percentage) {
    speed_limit_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else if (max_linear_speed_ > 0.0) {
    speed_limit_scale_ = std::clamp(speed_limit / max_linear_speed_, 0.0, 1.0);
  }
}

geometry_msgs::msg::PoseStamped DiffusionController::getLookaheadPointInBaseFrame(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  geometry_msgs::msg::PoseStamped selected;
  selected.header = global_plan_.header;
  selected.pose = global_plan_.poses.back().pose;

  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  for (const auto & stamped : global_plan_.poses) {
    const double dx = stamped.pose.position.x - rx;
    const double dy = stamped.pose.position.y - ry;
    if (std::hypot(dx, dy) >= lookahead_distance_) {
      selected.pose = stamped.pose;
      break;
    }
  }

  geometry_msgs::msg::PoseStamped carrot;
  tf_->transform(
    selected, carrot, base_frame_, tf2::durationFromSec(transform_tolerance_));
  return carrot;
}

nav2_diffusion_core::Trajectory DiffusionController::rollout(
  double linear, double angular) const
{
  nav2_diffusion_core::Trajectory trajectory;
  const int steps = std::max(1, static_cast<int>(horizon_ / time_step_));

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  trajectory.points.push_back({x, y, yaw, 0.0});
  for (int i = 1; i <= steps; ++i) {
    yaw += angular * time_step_;
    x += linear * std::cos(yaw) * time_step_;
    y += linear * std::sin(yaw) * time_step_;
    trajectory.points.push_back({x, y, yaw, i * time_step_});
  }
  return trajectory;
}

nav2_diffusion_core::Trajectory DiffusionController::toGlobalFrame(
  const nav2_diffusion_core::Trajectory & base_trajectory,
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  const auto & q = robot_pose.pose.orientation;
  const double yaw0 =
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos0 = std::cos(yaw0);
  const double sin0 = std::sin(yaw0);
  const double x0 = robot_pose.pose.position.x;
  const double y0 = robot_pose.pose.position.y;

  nav2_diffusion_core::Trajectory global;
  global.model_score = base_trajectory.model_score;
  global.points.reserve(base_trajectory.points.size());
  for (const auto & p : base_trajectory.points) {
    nav2_diffusion_core::TrajectoryPoint gp;
    gp.x = x0 + p.x * cos0 - p.y * sin0;
    gp.y = y0 + p.x * sin0 + p.y * cos0;
    gp.yaw = yaw0 + p.yaw;
    gp.time = p.time;
    global.points.push_back(gp);
  }
  return global;
}

geometry_msgs::msg::TwistStamped DiffusionController::makeStopCommand() const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();
  return cmd;
}

geometry_msgs::msg::TwistStamped DiffusionController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::FALLBACK, "no global plan");
    return makeStopCommand();
  }

  geometry_msgs::msg::PoseStamped carrot;
  try {
    carrot = getLookaheadPointInBaseFrame(pose);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(logger_, "Lookahead transform failed: %s", ex.what());
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::DEGRADED, "tf failure");
    return makeStopCommand();
  }

  // Placeholder generative proposal: pure-pursuit command toward the carrot.
  const double dx = carrot.pose.position.x;
  const double dy = carrot.pose.position.y;
  const double dist_sq = dx * dx + dy * dy;
  if (dist_sq < 1e-6) {
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::NOMINAL, "at goal");
    return makeStopCommand();
  }

  double linear = std::min(desired_linear_speed_, max_linear_speed_) * speed_limit_scale_;
  const double curvature = 2.0 * dy / dist_sq;
  double angular = std::clamp(linear * curvature, -max_angular_speed_, max_angular_speed_);

  const nav2_diffusion_core::Trajectory candidate = rollout(linear, angular);

  // Deterministic safety gate: kinematic limits first (cheap), then footprint
  // collision against the live costmap (docs/safety.md section 8.2).
  nav2_diffusion_safety::SafetyResult verdict = kinematic_filter_->check(candidate);
  if (verdict.safe) {
    const nav2_diffusion_core::Trajectory global_candidate = toGlobalFrame(candidate, pose);
    auto * costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    footprint_filter_->setCostmap(costmap);
    footprint_filter_->setFootprint(costmap_ros_->getRobotFootprint());
    verdict = footprint_filter_->check(global_candidate);
  }

  publishCandidates(candidate, verdict.safe, verdict.rejection_reason);

  if (!verdict.safe) {
    RCLCPP_WARN(
      logger_, "No safe candidate (%s); stopping", verdict.rejection_reason.c_str());
    publishSafetyState(
      nav2_diffusion_msgs::msg::SafetyState::BRAKE, verdict.rejection_reason);
    return makeStopCommand();
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();
  cmd.twist.linear.x = linear;
  cmd.twist.angular.z = angular;
  publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::NOMINAL, "");
  return cmd;
}

void DiffusionController::publishCandidates(
  const nav2_diffusion_core::Trajectory & trajectory,
  bool safe, const std::string & rejection_reason) const
{
  if (!candidates_pub_ || candidates_pub_->get_subscription_count() == 0) {
    return;
  }

  nav2_diffusion_msgs::msg::TrajectoryCandidate candidate;
  candidate.header.frame_id = base_frame_;
  candidate.header.stamp = clock_->now();
  candidate.model_score = trajectory.model_score;
  for (const auto & point : trajectory.points) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, point.yaw);
    pose.orientation = tf2::toMsg(q);
    candidate.poses.push_back(pose);

    builtin_interfaces::msg::Duration offset;
    offset.sec = static_cast<int32_t>(point.time);
    offset.nanosec = static_cast<uint32_t>((point.time - offset.sec) * 1e9);
    candidate.time_offsets.push_back(offset);
  }

  nav2_diffusion_msgs::msg::TrajectoryCandidates msg;
  msg.header = candidate.header;
  msg.candidates.push_back(candidate);
  msg.safe_flags.push_back(safe);
  msg.rejection_reasons.push_back(rejection_reason);
  msg.best_index = safe ? 0 : -1;
  candidates_pub_->publish(std::move(msg));
}

void DiffusionController::publishSafetyState(uint8_t state, const std::string & detail) const
{
  if (!safety_pub_) {
    return;
  }
  nav2_diffusion_msgs::msg::SafetyState msg;
  msg.header.frame_id = base_frame_;
  msg.header.stamp = clock_->now();
  msg.state = state;
  msg.detail = detail;
  safety_pub_->publish(std::move(msg));
}

}  // namespace nav2_diffusion_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_diffusion_controller::DiffusionController, nav2_core::Controller)
