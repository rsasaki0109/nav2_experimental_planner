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
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_diffusion_core/fan_rollout_model.hpp"
#include "nav2_diffusion_core/scoring.hpp"
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
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".data_timeout", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".check_costmap_current", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".num_candidates", rclcpp::ParameterValue(11));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".score_progress_weight", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".score_smoothness_weight", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".fallback_controller_plugin", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".model_plugin", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".model_path", rclcpp::ParameterValue(std::string("")));

  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".desired_linear_speed", desired_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node->get_parameter(plugin_name_ + ".horizon", horizon_);
  node->get_parameter(plugin_name_ + ".time_step", time_step_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".consider_unknown_lethal", consider_unknown_lethal_);
  node->get_parameter(plugin_name_ + ".data_timeout", data_timeout_);
  node->get_parameter(plugin_name_ + ".check_costmap_current", check_costmap_current_);
  node->get_parameter(plugin_name_ + ".num_candidates", num_candidates_);
  node->get_parameter(plugin_name_ + ".score_progress_weight", score_progress_weight_);
  node->get_parameter(plugin_name_ + ".score_smoothness_weight", score_smoothness_weight_);
  node->get_parameter(plugin_name_ + ".fallback_controller_plugin", fallback_plugin_);
  node->get_parameter(plugin_name_ + ".model_plugin", model_plugin_);
  node->get_parameter(plugin_name_ + ".model_path", model_path_);
  num_candidates_ = std::max(1, num_candidates_);

  // Generative model: a pluginlib-loaded TrajectoryModel (e.g. an ONNX backend)
  // when model_plugin is set, otherwise the built-in FanRolloutModel. The
  // controller does not link any inference backend; it loads one at runtime.
  if (!model_plugin_.empty()) {
    try {
      model_loader_ =
        std::make_unique<pluginlib::ClassLoader<nav2_diffusion_core::TrajectoryModel>>(
        "nav2_diffusion_core", "nav2_diffusion_core::TrajectoryModel");
      model_ = model_loader_->createSharedInstance(model_plugin_);
      model_->configure(model_path_);
      RCLCPP_INFO(logger_, "Loaded trajectory model plugin '%s'", model_plugin_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to load model plugin '%s': %s; using built-in FanRolloutModel",
        model_plugin_.c_str(), ex.what());
      model_ = std::make_shared<nav2_diffusion_core::FanRolloutModel>();
    }
  } else {
    model_ = std::make_shared<nav2_diffusion_core::FanRolloutModel>();
  }
  kinematic_filter_ = std::make_shared<nav2_diffusion_safety::KinematicLimitsFilter>(
    max_linear_speed_, max_angular_speed_);
  footprint_filter_ = std::make_shared<nav2_diffusion_safety::FootprintCollisionFilter>(
    costmap_ros_->getCostmap(), costmap_ros_->getRobotFootprint(),
    253.0, consider_unknown_lethal_);

  if (!fallback_plugin_.empty()) {
    try {
      fallback_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
        "nav2_core", "nav2_core::Controller");
      fallback_controller_ = fallback_loader_->createSharedInstance(fallback_plugin_);
      fallback_controller_->configure(parent, plugin_name_ + ".fallback", tf_, costmap_ros_);
      RCLCPP_INFO(logger_, "Loaded fallback controller '%s'", fallback_plugin_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to load fallback controller '%s': %s; will stop instead",
        fallback_plugin_.c_str(), ex.what());
      fallback_controller_.reset();
      fallback_loader_.reset();
    }
  }

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
  model_.reset();
  model_loader_.reset();
  kinematic_filter_.reset();
  footprint_filter_.reset();
  if (fallback_controller_) {
    fallback_controller_->cleanup();
    fallback_controller_.reset();
  }
  fallback_loader_.reset();
}

void DiffusionController::activate()
{
  candidates_pub_->on_activate();
  safety_pub_->on_activate();
  if (fallback_controller_) {
    fallback_controller_->activate();
  }
}

void DiffusionController::deactivate()
{
  candidates_pub_->on_deactivate();
  safety_pub_->on_deactivate();
  if (fallback_controller_) {
    fallback_controller_->deactivate();
  }
}

void DiffusionController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (fallback_controller_) {
    fallback_controller_->setPlan(path);
  }
}

void DiffusionController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (fallback_controller_) {
    fallback_controller_->setSpeedLimit(speed_limit, percentage);
  }
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

geometry_msgs::msg::Twist DiffusionController::extractCommand(
  const nav2_diffusion_core::Trajectory & trajectory) const
{
  geometry_msgs::msg::Twist cmd;
  if (trajectory.points.size() < 2) {
    return cmd;
  }
  const auto & p0 = trajectory.points.front();
  const auto & p1 = trajectory.points[1];
  const double dt = p1.time - p0.time;
  if (dt <= 0.0) {
    return cmd;
  }
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  double linear = std::hypot(dx, dy) / dt;
  // Sign: negative if the first step moves behind the starting heading.
  if (dx * std::cos(p0.yaw) + dy * std::sin(p0.yaw) < 0.0) {
    linear = -linear;
  }
  double dyaw = std::fmod(p1.yaw - p0.yaw + M_PI, 2.0 * M_PI);
  if (dyaw < 0.0) {
    dyaw += 2.0 * M_PI;
  }
  dyaw -= M_PI;

  cmd.linear.x = linear;
  cmd.angular.z = dyaw / dt;
  return cmd;
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
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (global_plan_.poses.empty()) {
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::FALLBACK, "no global plan");
    return makeStopCommand();
  }

  // Input-validity / stale-data gate (docs/architecture.md 7.4 Runtime Gating,
  // docs/safety.md 8.2 Input Validity Layer). A timestamped-but-stale robot pose
  // means odom/TF are no longer trustworthy, so we stop rather than act on it.
  if (data_timeout_ > 0.0) {
    const rclcpp::Time stamp(pose.header.stamp, clock_->get_clock_type());
    if (stamp.nanoseconds() > 0) {
      const double age = (clock_->now() - stamp).seconds();
      if (age > data_timeout_) {
        RCLCPP_WARN(logger_, "Robot pose is stale (%.3fs > %.3fs); stopping", age, data_timeout_);
        publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::DEGRADED, "stale pose");
        return makeStopCommand();
      }
    }
  }

  if (check_costmap_current_ && !costmap_ros_->isCurrent()) {
    RCLCPP_WARN(logger_, "Costmap is not current; stopping");
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::DEGRADED, "costmap not current");
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

  const double dx = carrot.pose.position.x;
  const double dy = carrot.pose.position.y;
  if (dx * dx + dy * dy < 1e-6) {
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::NOMINAL, "at goal");
    return makeStopCommand();
  }

  // Multimodal proposal: ask the generative model for K candidate trajectories
  // (base frame). The built-in FanRolloutModel is a placeholder for a learned
  // model behind the same nav2_diffusion_core::TrajectoryModel interface.
  nav2_diffusion_core::ModelContext context;
  context.goal_x = dx;
  context.goal_y = dy;
  context.linear_speed = std::min(desired_linear_speed_, max_linear_speed_) * speed_limit_scale_;
  context.max_angular_speed = max_angular_speed_;
  context.horizon = horizon_;
  context.time_step = time_step_;
  context.num_candidates = num_candidates_;
  const std::vector<nav2_diffusion_core::Trajectory> candidates = model_->generate(context);

  std::vector<bool> safe_flags(candidates.size(), false);
  std::vector<std::string> rejection_reasons(candidates.size());

  // Deterministic safety gate per candidate: kinematic limits (cheap) first,
  // then footprint collision against the live costmap under its mutex.
  {
    auto * costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    footprint_filter_->setCostmap(costmap);
    footprint_filter_->setFootprint(costmap_ros_->getRobotFootprint());
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      auto verdict = kinematic_filter_->check(candidates[i]);
      if (verdict.safe) {
        verdict = footprint_filter_->check(toGlobalFrame(candidates[i], pose));
      }
      safe_flags[i] = verdict.safe;
      rejection_reasons[i] = verdict.rejection_reason;
    }
  }

  // Soft scoring: among safe candidates pick the one that reaches the carrot
  // most directly and smoothly (docs/architecture.md section 4.1 step 7).
  const nav2_diffusion_core::ScoringWeights weights{
    score_progress_weight_, score_smoothness_weight_};
  int best_index = -1;
  double best_score = 0.0;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    if (!safe_flags[i]) {
      continue;
    }
    const double score = nav2_diffusion_core::scoreTrajectory(candidates[i], dx, dy, weights);
    if (best_index < 0 || score > best_score) {
      best_index = static_cast<int>(i);
      best_score = score;
    }
  }

  publishCandidates(candidates, safe_flags, rejection_reasons, best_index);

  if (best_index < 0) {
    if (fallback_controller_) {
      RCLCPP_WARN(
        logger_, "No safe generative candidate among %zu; delegating to fallback '%s'",
        candidates.size(), fallback_plugin_.c_str());
      publishSafetyState(
        nav2_diffusion_msgs::msg::SafetyState::FALLBACK, "fallback: " + fallback_plugin_);
      return fallback_controller_->computeVelocityCommands(pose, velocity, goal_checker);
    }
    RCLCPP_WARN(logger_, "No safe candidate among %zu; stopping", candidates.size());
    publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::BRAKE, "no safe candidate");
    return makeStopCommand();
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = base_frame_;
  cmd.header.stamp = clock_->now();
  cmd.twist = extractCommand(candidates[best_index]);
  publishSafetyState(nav2_diffusion_msgs::msg::SafetyState::NOMINAL, "");
  return cmd;
}

void DiffusionController::publishCandidates(
  const std::vector<nav2_diffusion_core::Trajectory> & candidates,
  const std::vector<bool> & safe_flags,
  const std::vector<std::string> & rejection_reasons,
  int best_index) const
{
  if (!candidates_pub_ || candidates_pub_->get_subscription_count() == 0) {
    return;
  }

  nav2_diffusion_msgs::msg::TrajectoryCandidates msg;
  msg.header.frame_id = base_frame_;
  msg.header.stamp = clock_->now();
  msg.best_index = best_index;
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    nav2_diffusion_msgs::msg::TrajectoryCandidate candidate;
    candidate.header = msg.header;
    candidate.model_score = candidates[i].model_score;
    for (const auto & point : candidates[i].points) {
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
    msg.candidates.push_back(candidate);
    msg.safe_flags.push_back(i < safe_flags.size() ? safe_flags[i] : false);
    msg.rejection_reasons.push_back(i < rejection_reasons.size() ? rejection_reasons[i] : "");
  }
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
