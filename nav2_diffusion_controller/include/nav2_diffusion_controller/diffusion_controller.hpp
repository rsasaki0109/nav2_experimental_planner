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

#ifndef NAV2_DIFFUSION_CONTROLLER__DIFFUSION_CONTROLLER_HPP_
#define NAV2_DIFFUSION_CONTROLLER__DIFFUSION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_diffusion_core/trajectory.hpp"
#include "nav2_diffusion_core/trajectory_model.hpp"
#include "nav2_diffusion_msgs/msg/safety_state.hpp"
#include "nav2_diffusion_msgs/msg/trajectory_candidates.hpp"
#include "nav2_diffusion_safety/footprint_collision_filter.hpp"
#include "nav2_diffusion_safety/kinematic_limits_filter.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_diffusion_controller
{

/// Nav2 Controller plugin (v0.1 skeleton) for generative local planning.
///
/// Implements the pipeline described in docs/architecture.md sections 3.2
/// (Mode A) and 4.1: a generative model proposes candidate trajectories, a
/// deterministic safety gate validates them, and cmd_vel is extracted from the
/// best safe candidate. When no safe candidate exists the controller stops as a
/// fallback (docs/safety.md section 8.4).
///
/// The generative model is a placeholder here (a single pure-pursuit-style
/// candidate toward a lookahead point). It exists so the Nav2 integration,
/// safety wiring, and observable outputs can be built and tested before a real
/// learned model is plugged in.
class DiffusionController : public nav2_core::Controller
{
public:
  DiffusionController() = default;
  ~DiffusionController() override = default;

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
  /// Select a lookahead point on the global plan and express it in the robot
  /// base frame. Throws tf2::TransformException on transform failure.
  geometry_msgs::msg::PoseStamped getLookaheadPointInBaseFrame(
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  /// Express a base-frame trajectory in the costmap global frame using the
  /// robot's current global pose (for footprint collision checking).
  nav2_diffusion_core::Trajectory toGlobalFrame(
    const nav2_diffusion_core::Trajectory & base_trajectory,
    const geometry_msgs::msg::PoseStamped & robot_pose) const;

  /// Extract the velocity command from the first segment of a base-frame
  /// trajectory (docs/architecture.md section 4.4: cmd_vel comes from the best
  /// trajectory, not from a stored control).
  geometry_msgs::msg::Twist extractCommand(
    const nav2_diffusion_core::Trajectory & trajectory) const;

  /// Publish all candidates (with per-candidate safety verdict and the selected
  /// best index) for RViz / rosbag observability.
  void publishCandidates(
    const std::vector<nav2_diffusion_core::Trajectory> & candidates,
    const std::vector<bool> & safe_flags,
    const std::vector<std::string> & rejection_reasons,
    int best_index) const;

  void publishSafetyState(uint8_t state, const std::string & detail) const;

  geometry_msgs::msg::TwistStamped makeStopCommand() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("DiffusionController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::string base_frame_;

  nav_msgs::msg::Path global_plan_;

  double lookahead_distance_{0.6};
  double desired_linear_speed_{0.3};
  double max_linear_speed_{0.5};
  double max_angular_speed_{1.0};
  double horizon_{2.0};
  double time_step_{0.1};
  double transform_tolerance_{0.1};
  double speed_limit_scale_{1.0};
  bool consider_unknown_lethal_{false};
  double data_timeout_{0.5};
  bool check_costmap_current_{false};
  int num_candidates_{11};
  double score_progress_weight_{1.0};
  double score_smoothness_weight_{0.1};

  std::shared_ptr<nav2_diffusion_core::TrajectoryModel> model_;
  std::shared_ptr<nav2_diffusion_safety::KinematicLimitsFilter> kinematic_filter_;
  std::shared_ptr<nav2_diffusion_safety::FootprintCollisionFilter> footprint_filter_;

  // Optional fallback controller (e.g. MPPI / RPP). When no safe generative
  // candidate exists, control is delegated here instead of stopping
  // (docs/safety.md section 8.4). Empty plugin string disables it.
  std::string fallback_plugin_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> fallback_loader_;
  nav2_core::Controller::Ptr fallback_controller_;

  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<nav2_diffusion_msgs::msg::TrajectoryCandidates>>
  candidates_pub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<nav2_diffusion_msgs::msg::SafetyState>>
  safety_pub_;
};

}  // namespace nav2_diffusion_controller

#endif  // NAV2_DIFFUSION_CONTROLLER__DIFFUSION_CONTROLLER_HPP_
