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

#ifndef NAV2_VFH_CONTROLLER__VFH_CONTROLLER_HPP_
#define NAV2_VFH_CONTROLLER__VFH_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_vfh_controller
{

/// Vector Field Histogram Plus (VFH+) local controller as a Nav2
/// nav2_core::Controller. VFH+ (Ulrich & Borenstein, 1998) builds a polar
/// histogram of obstacle density around the robot, reduces it to a binary
/// histogram of blocked / free angular sectors (with each obstacle enlarged by
/// the robot radius so a free sector is actually traversable), and steers toward
/// the free sector ("valley") that best balances heading to the goal against
/// turning effort and steering smoothness. Unlike Nav2's optimization-based local
/// controllers (DWB, MPPI, Regulated Pure Pursuit), VFH+ is a reactive
/// histogram method — cheap and robust in cluttered space, with no trajectory
/// rollout. Upstream Nav2 has no VFH controller. Classical (non-learned) and
/// deterministic.
class VFHController : public nav2_core::Controller
{
public:
  VFHController() = default;
  ~VFHController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /// Bearing (robot frame, radians) to a lookahead point on the global plan.
  double lookaheadBearing(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  /// Build the binary polar histogram (true = blocked sector) around the robot.
  std::vector<char> buildHistogram(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  /// Centre angle (radians, robot frame) of sector k.
  double sectorAngle(int k) const;
  geometry_msgs::msg::TwistStamped makeStopCommand() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("VFHController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::string base_frame_;

  nav_msgs::msg::Path global_plan_;
  double prev_direction_{0.0};  // last chosen steering angle (robot frame) [rad]

  // Parameters
  int num_sectors_{72};
  double active_window_{2.0};      // histogram radius around the robot [m]
  int obstacle_threshold_{253};    // costs >= this count as obstacles
  bool allow_unknown_{true};
  double robot_radius_{0.2};       // for obstacle enlargement [m]
  double safety_distance_{0.1};    // extra enlargement margin [m]
  int min_valley_sectors_{1};      // required free width around a candidate
  double lookahead_distance_{0.6};
  double max_linear_speed_{0.5};
  double max_angular_speed_{1.5};
  double angular_gain_{1.5};
  double goal_dist_tolerance_{0.25};
  double transform_tolerance_{0.1};
  double mu_target_{5.0};   // cost weight: alignment with the goal direction
  double mu_heading_{2.0};  // cost weight: turning away from straight ahead
  double mu_smooth_{2.0};   // cost weight: change from the previous direction
  double speed_limit_scale_{1.0};
};

}  // namespace nav2_vfh_controller

#endif  // NAV2_VFH_CONTROLLER__VFH_CONTROLLER_HPP_
