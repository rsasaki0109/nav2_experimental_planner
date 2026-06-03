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

#ifndef NAV2_DIFFUSION_GLOBAL_PLANNER__DIFFUSION_GLOBAL_PLANNER_HPP_
#define NAV2_DIFFUSION_GLOBAL_PLANNER__DIFFUSION_GLOBAL_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_diffusion_core/fan_path_model.hpp"
#include "nav2_diffusion_core/path.hpp"
#include "nav2_diffusion_core/path_model.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_diffusion_global_planner
{

/// Generative global planner (Nav2 Mode B): a model proposes K candidate
/// start->goal paths; a deterministic validity layer checks each against the
/// global costmap and the shortest collision-free one is returned. When no
/// candidate is collision-free, throws NoValidPathCouldBeFound so the planner
/// server can recover/replan.
///
/// The proposal stage is a nav2_diffusion_core::PathModel. The default is the
/// built-in analytic FanPathModel; set `model_plugin` to load a learned model
/// (e.g. an ONNX-backed PathModel) at runtime via pluginlib, exactly mirroring
/// the controller's TrajectoryModel seam. No open-source generative model is
/// currently integrated as a nav2_core::GlobalPlanner; this is that seam.
class DiffusionGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  DiffusionGlobalPlanner() = default;
  ~DiffusionGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  /// True if every densified segment of the path stays in traversable cells.
  bool isPathValid(const nav2_diffusion_core::PathCandidate & path) const;

  /// True if the world point is in-bounds and below the lethal cost threshold.
  bool isCellTraversable(double wx, double wy) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("DiffusionGlobalPlanner")};

  // Parameters
  int num_candidates_{11};
  int num_points_{40};
  double interpolation_resolution_{0.05};
  bool allow_unknown_{true};
  double max_bow_fraction_{0.5};
  std::string model_plugin_;
  std::string model_path_;

  std::unique_ptr<pluginlib::ClassLoader<nav2_diffusion_core::PathModel>> model_loader_;
  std::shared_ptr<nav2_diffusion_core::PathModel> model_;
};

}  // namespace nav2_diffusion_global_planner

#endif  // NAV2_DIFFUSION_GLOBAL_PLANNER__DIFFUSION_GLOBAL_PLANNER_HPP_
