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

#ifndef NAV2_DIFFUSION_GLOBAL_PLANNER__DIFFUSION_GLOBAL_PLANNER_HPP_
#define NAV2_DIFFUSION_GLOBAL_PLANNER__DIFFUSION_GLOBAL_PLANNER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

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
/// candidate is collision-free, it delegates to an optional classical fallback
/// planner (a complete search) if `fallback_planner_plugin` is set, otherwise it
/// throws NoValidPathCouldBeFound so the planner server can recover/replan.
///
/// The proposal stage is a nav2_diffusion_core::PathModel. The default is the
/// built-in analytic FanPathModel; set `model_plugin` to load a learned model
/// (e.g. an ONNX-backed PathModel) at runtime via pluginlib, exactly mirroring
/// the controller's TrajectoryModel seam. No open-source generative model is
/// currently integrated as a nav2_core::GlobalPlanner; this is that seam.
///
/// `fallback_planner_plugin` makes this a *hybrid* planner: the learned model
/// proposes (fast, multimodal, costmap-biased) and a classical search disposes by
/// guaranteeing a complete path when no proposal threads the map — so it keeps the
/// generative proposal on easy maps but never regresses below a search planner on
/// hard ones (e.g. routing through an off-centre gap). See docs/generative_limits.md.
///
/// `hybrid_mode: guided` is the tightly-coupled variant: instead of only handing
/// off on failure, it always runs a built-in A* search but discounts the cost of
/// cells near the valid generative proposals, so the learned model *shapes* every
/// plan (which way to go around) while the search guarantees completeness. With no
/// valid proposal it degrades to a plain complete A*.
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

  /// Copy the global costmap (normalized [0, 1]) + geometry into the context so
  /// costmap-conditioned PathModels can read it.
  void fillCostmap(nav2_diffusion_core::PathContext & ctx) const;

  /// Tightly-coupled hybrid: a complete 8-connected A* over the costmap whose
  /// per-cell cost is discounted near the valid generative proposals, so the
  /// learned model shapes the route while the search guarantees completeness.
  /// Returns an empty path if no route exists.
  nav_msgs::msg::Path guidedSearch(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<nav2_diffusion_core::PathCandidate> & proposals,
    std::function<bool()> cancel_checker) const;

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
  bool provide_costmap_{true};
  std::string model_plugin_;
  std::string model_path_;
  std::string fallback_planner_plugin_;
  std::string hybrid_mode_{"fallback"};   // "fallback" or "guided"
  double guidance_strength_{0.5};          // cost discount near proposals [0, 1)
  double guidance_radius_{0.3};            // corridor half-width around proposals [m]
  double min_turn_radius_{0.0};            // vehicle min turn radius R [m]; 0 = no kinematic limit

  std::unique_ptr<pluginlib::ClassLoader<nav2_diffusion_core::PathModel>> model_loader_;
  std::shared_ptr<nav2_diffusion_core::PathModel> model_;

  // Optional classical fallback (a complete search) used when no generative
  // candidate is collision-free — makes this a hybrid propose/search planner.
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> fallback_loader_;
  std::shared_ptr<nav2_core::GlobalPlanner> fallback_planner_;
};

}  // namespace nav2_diffusion_global_planner

#endif  // NAV2_DIFFUSION_GLOBAL_PLANNER__DIFFUSION_GLOBAL_PLANNER_HPP_
