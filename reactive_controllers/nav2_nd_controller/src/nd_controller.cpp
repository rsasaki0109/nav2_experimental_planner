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

#include "nav2_nd_controller/nd_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_nd_controller
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kInf = std::numeric_limits<double>::infinity();

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

void NDController::configure(
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
    node, name + ".security_distance", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, name + ".min_region_sectors", rclcpp::ParameterValue(3));
  declare_parameter_if_not_declared(
    node, name + ".lookahead_distance", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, name + ".max_linear_speed", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, name + ".max_angular_speed", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(node, name + ".angular_gain", rclcpp::ParameterValue(1.5));
  declare_parameter_if_not_declared(
    node, name + ".deflection_gain", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name + ".slow_distance", rclcpp::ParameterValue(0.6));
  declare_parameter_if_not_declared(
    node, name + ".goal_dist_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(
    node, name + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(name + ".num_sectors", num_sectors_);
  node->get_parameter(name + ".active_window", active_window_);
  node->get_parameter(name + ".obstacle_threshold", obstacle_threshold_);
  node->get_parameter(name + ".allow_unknown", allow_unknown_);
  node->get_parameter(name + ".robot_radius", robot_radius_);
  node->get_parameter(name + ".safety_distance", safety_distance_);
  node->get_parameter(name + ".security_distance", security_distance_);
  node->get_parameter(name + ".min_region_sectors", min_region_sectors_);
  node->get_parameter(name + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(name + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(name + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(name + ".angular_gain", angular_gain_);
  node->get_parameter(name + ".deflection_gain", deflection_gain_);
  node->get_parameter(name + ".slow_distance", slow_distance_);
  node->get_parameter(name + ".goal_dist_tolerance", goal_dist_tolerance_);
  node->get_parameter(name + ".transform_tolerance", transform_tolerance_);

  if (num_sectors_ < 8) {
    num_sectors_ = 8;
  }

  RCLCPP_INFO(
    logger_, "NDController '%s' configured: num_sectors=%d security_distance=%.2f",
    plugin_name_.c_str(), num_sectors_, security_distance_);
}

void NDController::cleanup() {}
void NDController::activate() {}
void NDController::deactivate() {}

void NDController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void NDController::setSpeedLimit(const double & speed_limit, const bool & percentage)
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

double NDController::sectorAngle(int k) const
{
  const double alpha = kTwoPi / num_sectors_;
  return -M_PI + (k + 0.5) * alpha;
}

double NDController::lookaheadBearing(const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  // Nearest plan pose, then look ahead forward from there (avoids latching onto
  // already-passed poses behind the robot, which would point the carrot back).
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  std::size_t nearest = 0;
  double nearest_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < global_plan_.poses.size(); ++i) {
    const double d = std::hypot(
      global_plan_.poses[i].pose.position.x - rx,
      global_plan_.poses[i].pose.position.y - ry);
    if (d < nearest_d) {
      nearest_d = d;
      nearest = i;
    }
  }
  geometry_msgs::msg::PoseStamped selected;
  selected.header = global_plan_.header;
  selected.pose = global_plan_.poses.back().pose;
  for (std::size_t i = nearest; i < global_plan_.poses.size(); ++i) {
    const double dx = global_plan_.poses[i].pose.position.x - rx;
    const double dy = global_plan_.poses[i].pose.position.y - ry;
    if (std::hypot(dx, dy) >= lookahead_distance_) {
      selected.pose = global_plan_.poses[i].pose;
      break;
    }
  }
  geometry_msgs::msg::PoseStamped carrot;
  tf_->transform(
    selected, carrot, base_frame_, tf2::durationFromSec(transform_tolerance_));
  return std::atan2(carrot.pose.position.y, carrot.pose.position.x);
}

NDController::Diagram NDController::buildDiagram(
  const geometry_msgs::msg::PoseStamped & robot_pose) const
{
  Diagram diag;
  diag.distance.assign(num_sectors_, kInf);
  auto * costmap = costmap_ros_->getCostmap();
  const double alpha = kTwoPi / num_sectors_;
  const double rx = robot_pose.pose.position.x;
  const double ry = robot_pose.pose.position.y;
  const double ryaw = yawFromQuaternion(robot_pose.pose.orientation);
  const double front_cone = M_PI / 6.0;  // +/- 30 deg counts as "ahead"

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
      int k = static_cast<int>(std::floor((bearing + M_PI) / alpha));
      k = std::clamp(k, 0, num_sectors_ - 1);
      diag.distance[k] = std::min(diag.distance[k], d);

      if (d < diag.closest) {
        diag.closest = d;
      }
      if (std::abs(bearing) < front_cone) {
        diag.front_distance = std::min(diag.front_distance, d);
      }
      // Symmetric safety deflection: a close obstacle on one side pushes the
      // robot toward the other, so balanced obstacles centre it in a corridor.
      if (d < security_distance_) {
        const double demand = (security_distance_ - d) / security_distance_;
        if (bearing < 0.0) {
          diag.push_left = std::max(diag.push_left, demand);   // obstacle right -> go left
        } else if (bearing > 0.0) {
          diag.push_right = std::max(diag.push_right, demand);  // obstacle left -> go right
        }
      }
    }
  }
  return diag;
}

geometry_msgs::msg::TwistStamped NDController::makeStopCommand() const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();
  return cmd;
}

geometry_msgs::msg::TwistStamped NDController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  if (global_plan_.poses.empty()) {
    return makeStopCommand();
  }
  const auto & last = global_plan_.poses.back().pose.position;
  const double goal_dist = std::hypot(
    last.x - pose.pose.position.x, last.y - pose.pose.position.y);
  if (goal_dist < goal_dist_tolerance_) {
    return makeStopCommand();
  }

  double target = 0.0;
  try {
    target = lookaheadBearing(pose);
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger_, "NDController lookahead transform failed: %s", e.what());
    return makeStopCommand();
  }

  const Diagram diag = buildDiagram(pose);
  const double alpha = kTwoPi / num_sectors_;
  // A sector is navigable if its nearest obstacle is beyond both the collision
  // clearance and the security distance, so a frontal obstacle blocks the sectors
  // ahead and the robot picks a side gap instead of driving into it.
  const double block_distance = std::max(robot_radius_ + safety_distance_, security_distance_);

  std::vector<char> navigable(num_sectors_, 0);
  for (int k = 0; k < num_sectors_; ++k) {
    navigable[k] = diag.distance[k] > block_distance ? 1 : 0;
  }
  const int w = std::max(0, (min_region_sectors_ - 1) / 2);
  auto regionFree = [&](int k) -> bool {
      for (int off = -w; off <= w; ++off) {
        int idx = (k + off) % num_sectors_;
        if (idx < 0) {
          idx += num_sectors_;
        }
        if (!navigable[idx]) {
          return false;
        }
      }
      return true;
    };

  int target_sector = static_cast<int>(std::floor((target + M_PI) / alpha));
  target_sector = std::clamp(target_sector, 0, num_sectors_ - 1);

  double region_dir = target;
  bool have_region = false;
  if (regionFree(target_sector)) {
    region_dir = target;  // goal lies in a navigable region: head straight for it
    have_region = true;
  } else {
    int best = -1;
    int best_dist = num_sectors_;
    for (int k = 0; k < num_sectors_; ++k) {
      if (!regionFree(k)) {
        continue;
      }
      int circ = std::abs(k - target_sector);
      circ = std::min(circ, num_sectors_ - circ);
      if (circ < best_dist) {
        best_dist = circ;
        best = k;
      }
    }
    if (best >= 0) {
      region_dir = sectorAngle(best);
      have_region = true;
    }
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();

  if (!have_region) {
    // No navigable gap: rotate toward the goal to search for an opening.
    cmd.twist.angular.z = std::clamp(
      angular_gain_ * target, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  // ND safety deflection: add the net push away from close obstacles.
  const double deflection = deflection_gain_ * (diag.push_left - diag.push_right);
  double desired = angleDiff(region_dir + deflection, 0.0);

  cmd.twist.angular.z = std::clamp(
    angular_gain_ * desired, -max_angular_speed_, max_angular_speed_);

  // Slow for turns, for close frontal obstacles, and near the goal; no reverse.
  const double turn_factor = std::max(0.0, 1.0 - std::abs(desired) / (M_PI / 2.0));
  double front_factor = 1.0;
  if (diag.front_distance < slow_distance_) {
    front_factor = std::clamp(diag.front_distance / std::max(slow_distance_, 1e-3), 0.0, 1.0);
  }
  double linear = max_linear_speed_ * speed_limit_scale_ * turn_factor * front_factor;
  linear = std::min(linear, std::max(0.0, goal_dist));
  cmd.twist.linear.x = std::max(0.0, linear);
  return cmd;
}

}  // namespace nav2_nd_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_nd_controller::NDController, nav2_core::Controller)
