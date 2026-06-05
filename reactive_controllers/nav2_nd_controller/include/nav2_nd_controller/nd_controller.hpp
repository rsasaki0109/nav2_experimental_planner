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

#ifndef NAV2_ND_CONTROLLER__ND_CONTROLLER_HPP_
#define NAV2_ND_CONTROLLER__ND_CONTROLLER_HPP_

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

namespace nav2_nd_controller
{

/// Nearness Diagram (ND) reactive local controller as a Nav2
/// nav2_core::Controller. ND navigation (Minguez & Montano, 2004) reasons about
/// the "nearness" of obstacles per angular sector, finds navigable regions
/// (gaps) free enough for the robot to pass, picks the gap leading toward the
/// goal, and — its hallmark distinct from VFH — applies a *safety deflection*
/// that steers away from whichever side has a close obstacle, so the robot
/// centres itself in corridors and squeezes through tight gaps. This is a
/// simplified ND (per-sector nearest distance + region/gap selection + symmetric
/// safety deflection). It is a different reactive paradigm from the histogram
/// valley-cost VFH+ controller in this repo, and upstream Nav2 has neither.
/// Classical (non-learned) and deterministic.
class NDController : public nav2_core::Controller
{
public:
  NDController() = default;
  ~NDController() override = default;

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
  /// Result of scanning the local costmap into a polar nearness diagram.
  struct Diagram
  {
    std::vector<double> distance;  // per-sector nearest obstacle distance [m] (inf if none)
    double closest{1e9};           // nearest obstacle distance over all sectors [m]
    double push_left{0.0};         // deflection demand pushing left (from right-side obstacles)
    double push_right{0.0};        // deflection demand pushing right (from left-side obstacles)
    double front_distance{1e9};    // nearest obstacle within +/- the front cone [m]
  };

  double lookaheadBearing(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  Diagram buildDiagram(const geometry_msgs::msg::PoseStamped & robot_pose) const;
  double sectorAngle(int k) const;
  geometry_msgs::msg::TwistStamped makeStopCommand() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("NDController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::string base_frame_;

  nav_msgs::msg::Path global_plan_;

  // Parameters
  int num_sectors_{72};
  double active_window_{2.0};
  int obstacle_threshold_{253};
  bool allow_unknown_{true};
  double robot_radius_{0.2};
  double safety_distance_{0.1};
  double security_distance_{0.4};  // obstacles closer than this trigger deflection
  int min_region_sectors_{3};      // min navigable width to count as a gap
  double lookahead_distance_{0.6};
  double max_linear_speed_{0.5};
  double max_angular_speed_{1.5};
  double angular_gain_{1.5};
  double deflection_gain_{1.0};
  double slow_distance_{0.6};       // front clearance below which linear is scaled
  double goal_dist_tolerance_{0.25};
  double transform_tolerance_{0.1};
  double speed_limit_scale_{1.0};
};

}  // namespace nav2_nd_controller

#endif  // NAV2_ND_CONTROLLER__ND_CONTROLLER_HPP_
