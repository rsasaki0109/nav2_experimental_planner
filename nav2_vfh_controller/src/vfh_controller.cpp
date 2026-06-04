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

#include "nav2_vfh_controller/vfh_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_vfh_controller
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

// Smallest signed difference a - b wrapped to [-pi, pi].
double angleDiff(double a, double b)
{
  double d = std::fmod(a - b + M_PI, kTwoPi);
  if (d < 0.0) {
    d += kTwoPi;
  }
  return d - M_PI;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

void VFHController::configure(
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
  declare_parameter_if_not_declared(node, name + ".num_sectors", rclcpp::ParameterValue(72));
  declare_parameter_if_not_declared(node, name + ".active_window", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, name + ".obstacle_threshold", rclcpp::ParameterValue(253));
  declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name + ".robot_radius", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(node, name + ".safety_distance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, name + ".min_valley_sectors", rclcpp::ParameterValue(1));
  declare_parameter_if_not_declared(
    node, name + ".lookahead_distance", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, name + ".max_linear_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, name + ".max_angular_speed", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name + ".angular_gain", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, name + ".goal_dist_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, name + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name + ".mu_target", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(node, name + ".mu_heading", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name + ".mu_smooth", rclcpp::ParameterValue(2.0));

  node->get_parameter(name + ".num_sectors", num_sectors_);
  node->get_parameter(name + ".active_window", active_window_);
  node->get_parameter(name + ".obstacle_threshold", obstacle_threshold_);
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  node->get_parameter(name + ".robot_radius", robot_radius_);
  node->get_parameter(name + ".safety_distance", safety_distance_);
  node->get_parameter(name + ".min_valley_sectors", min_valley_sectors_);
  node->get_parameter(name + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(name + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(name + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(name + ".angular_gain", angular_gain_);
  node->get_parameter(name + ".goal_dist_tolerance", goal_dist_tolerance_);
  node->get_parameter(name + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name + ".mu_target", mu_target_);
  node->get_parameter(name + ".mu_heading", mu_heading_);
  node->get_parameter(name + ".mu_smooth", mu_smooth_);

  if (num_sectors_ < 8) {
    num_sectors_ = 8;
  }

  RCLCPP_INFO(
    logger_, "VFHController '%s' configured: num_sectors=%d active_window=%.2f",
    plugin_name_.c_str(), num_sectors_, active_window_);
}

void VFHController::cleanup() {}
void VFHController::activate() {}
void VFHController::deactivate() {}

void VFHController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void VFHController::setSpeedLimit(const double & speed_limit, const bool & percentage)
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

double VFHController::sectorAngle(int k) const
{
  const double alpha = kTwoPi / num_sectors_;
  return -M_PI + (k + 0.5) * alpha;
}

double VFHController::lookaheadBearing(const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  // Pick the first plan pose at least lookahead_distance away (else the last).
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
  return std::atan2(carrot.pose.position.y, carrot.pose.position.x);
}

std::vector<char> VFHController::buildHistogram(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  std::vector<char> blocked(num_sectors_, 0);
  auto * costmap = costmap_ros_->getCostmap();
  const double alpha = kTwoPi / num_sectors_;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  const double ryaw = yawFromQuaternion(robot_pose.pose.orientation);
  const double enlarge = robot_radius_ + safety_distance_;

  int lo_x = 0;
  int lo_y = 0;
  int hi_x = 0;
  int hi_y = 0;
  costmap->worldToMapEnforceBounds(rx - active_window_, ry - active_window_, lo_x, lo_y);
  costmap->worldToMapEnforceBounds(rx + active_window_, ry + active_window_, hi_x, hi_y);
  const int max_x = static_cast<int>(costmap->getSizeInCellsX()) - 1;
  const int max_y = static_cast<int>(costmap->getSizeInCellsY()) - 1;
  lo_x = std::max(0, lo_x);
  lo_y = std::max(0, lo_y);
  hi_x = std::min(max_x, hi_x);
  hi_y = std::min(max_y, hi_y);

  for (int my = lo_y; my <= hi_y; ++my) {
    for (int mx = lo_x; mx <= hi_x; ++mx) {
      const unsigned char cost = costmap->getCost(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my));
      bool obstacle = false;
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        obstacle = !allow_unknown_;
      } else {
        obstacle = cost >= obstacle_threshold_;
      }
      if (!obstacle) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      costmap->mapToWorld(
        static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      const double dx = wx - rx;
      const double dy = wy - ry;
      const double d = std::hypot(dx, dy);
      if (d > active_window_ || d < 1e-6) {
        continue;
      }
      const double bearing = angleDiff(std::atan2(dy, dx), ryaw);
      // Enlarge the obstacle by the robot radius so a free sector is passable.
      const double ratio = std::min(1.0, enlarge / std::max(d, enlarge));
      const double gamma = std::asin(ratio);
      const int span = static_cast<int>(std::ceil(gamma / alpha));
      const int kc = static_cast<int>(std::floor((bearing + M_PI) / alpha));
      for (int off = -span; off <= span; ++off) {
        int k = (kc + off) % num_sectors_;
        if (k < 0) {
          k += num_sectors_;
        }
        blocked[k] = 1;
      }
    }
  }
  return blocked;
}

geometry_msgs::msg::TwistStamped VFHController::makeStopCommand() const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();
  return cmd;
}

geometry_msgs::msg::TwistStamped VFHController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    return makeStopCommand();
  }

  // Stop near the goal; the controller server's goal checker ends the action.
  const auto & last = global_plan_.poses.back().pose.position;
  const double goal_dist = std::hypot(
    last.x - pose.pose.position.x, last.y - pose.pose.position.y);
  if (goal_dist < goal_dist_tolerance_) {
    prev_direction_ = 0.0;
    return makeStopCommand();
  }

  double target = 0.0;
  try {
    target = lookaheadBearing(pose);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "VFHController lookahead transform failed: %s", e.what());
    return makeStopCommand();
  }

  const std::vector<char> blocked = buildHistogram(pose);
  const double alpha = kTwoPi / num_sectors_;
  const int w = std::max(0, (min_valley_sectors_ - 1) / 2);

  auto sectorFree = [&](int k) -> bool {
      for (int off = -w; off <= w; ++off) {
        int idx = (k + off) % num_sectors_;
        if (idx < 0) {
          idx += num_sectors_;
        }
        if (blocked[idx]) {
          return false;
        }
      }
      return true;
    };

  int target_sector = static_cast<int>(std::floor((target + M_PI) / alpha));
  target_sector = std::clamp(target_sector, 0, num_sectors_ - 1);

  double chosen = 0.0;
  bool have_choice = false;
  if (sectorFree(target_sector)) {
    chosen = target;  // straight at the goal direction
    have_choice = true;
  } else {
    double best_cost = std::numeric_limits<double>::max();
    for (int k = 0; k < num_sectors_; ++k) {
      if (!sectorFree(k)) {
        continue;
      }
      const double theta = sectorAngle(k);
      const double cost =
        mu_target_ * std::abs(angleDiff(theta, target)) +
        mu_heading_ * std::abs(theta) +
        mu_smooth_ * std::abs(angleDiff(theta, prev_direction_));
      if (cost < best_cost) {
        best_cost = cost;
        chosen = theta;
        have_choice = true;
      }
    }
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();

  if (!have_choice) {
    // Fully surrounded: rotate in place toward the goal to look for an opening.
    cmd.twist.angular.z = std::clamp(
      angular_gain_ * target, -max_angular_speed_, max_angular_speed_);
    prev_direction_ = 0.0;
    return cmd;
  }

  prev_direction_ = chosen;
  cmd.twist.angular.z = std::clamp(
    angular_gain_ * chosen, -max_angular_speed_, max_angular_speed_);
  // Slow down for turns and near the goal; never reverse.
  const double turn_factor = std::max(0.0, 1.0 - std::abs(chosen) / (M_PI / 2.0));
  double linear = max_linear_speed_ * speed_limit_scale_ * turn_factor;
  linear = std::min(linear, std::max(0.0, goal_dist));
  cmd.twist.linear.x = std::max(0.0, linear);
  return cmd;
}

}  // namespace nav2_vfh_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_vfh_controller::VFHController, nav2_core::Controller)
