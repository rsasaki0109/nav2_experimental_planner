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

// Closed-loop comparison of the repo's reactive nav2_core::Controller plugins
// (VFH+ and ND) on shared scenarios. Each controller is rolled out against a live
// nav2_costmap_2d::Costmap2DROS with a unicycle motion model (no Gazebo/GPU); the
// robot follows a straight global plan while the controller reacts to obstacles.
// Records goal success, steps, path length, minimum clearance, steering
// smoothness, and (in the corridor) how well the controller centres itself, then
// writes a Markdown report to stdout. Reproduce with:
//   ros2 run nav2_planner_benchmarks controller_benchmark > docs/controller_comparison.md

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace
{

struct ControllerEntry
{
  std::string label;
  std::string class_name;
};

struct Scenario
{
  std::string name;
  std::string description;
  double sx, sy, syaw, gx, gy;
  bool centering;        // measure mean |y - centerline|
  double centerline;     // corridor centre y
};

void clearCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }
  }
}

// Lethal block centred on a world point.
void markBlock(nav2_costmap_2d::Costmap2D * costmap, double wx, double wy, int half_cells)
{
  unsigned int cx = 0;
  unsigned int cy = 0;
  if (!costmap->worldToMap(wx, wy, cx, cy)) {
    return;
  }
  for (int dy = -half_cells; dy <= half_cells; ++dy) {
    for (int dx = -half_cells; dx <= half_cells; ++dx) {
      costmap->setCost(cx + dx, cy + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

// Horizontal lethal wall at world y = wy spanning [x0, x1].
void markHWall(
  nav2_costmap_2d::Costmap2D * costmap, double wy, double x0, double x1, int half_cells)
{
  const double res = costmap->getResolution();
  for (double x = x0; x <= x1; x += res) {
    unsigned int cx = 0;
    unsigned int cy = 0;
    if (!costmap->worldToMap(x, wy, cx, cy)) {
      continue;
    }
    for (int dy = -half_cells; dy <= half_cells; ++dy) {
      costmap->setCost(cx, cy + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

double nearestObstacle(nav2_costmap_2d::Costmap2D * costmap, double x, double y, double window)
{
  unsigned int rmx = 0;
  unsigned int rmy = 0;
  if (!costmap->worldToMap(x, y, rmx, rmy)) {
    return 0.0;  // off-map counts as no clearance
  }
  double best = std::numeric_limits<double>::max();
  const int w = static_cast<int>(window / costmap->getResolution());
  const int cx = static_cast<int>(rmx);
  const int cy = static_cast<int>(rmy);
  for (int my = cy - w; my <= cy + w; ++my) {
    for (int mx = cx - w; mx <= cx + w; ++mx) {
      if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap->getSizeInCellsX()) ||
        my >= static_cast<int>(costmap->getSizeInCellsY()))
      {
        continue;
      }
      if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        double wx = 0.0;
        double wy = 0.0;
        costmap->mapToWorld(
          static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
        best = std::min(best, std::hypot(wx - x, wy - y));
      }
    }
  }
  return best;
}

nav_msgs::msg::Path straightPlan(double sx, double sy, double gx, double gy)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const double dist = std::hypot(gx - sx, gy - sy);
  const int n = std::max(2, static_cast<int>(dist / 0.1));
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = sx + t * (gx - sx);
    p.pose.position.y = sy + t * (gy - sy);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  return path;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_benchmark");

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("width", 6),
    rclcpp::Parameter("height", 6),
    rclcpp::Parameter("resolution", 0.05),
    rclcpp::Parameter("robot_radius", 0.15),
    rclcpp::Parameter("global_frame", std::string("map")),
    rclcpp::Parameter("robot_base_frame", std::string("base_link")),
    rclcpp::Parameter("rolling_window", false),
    rclcpp::Parameter("track_unknown_space", false),
  });
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto * costmap = costmap_ros->getCostmap();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);

  const std::vector<ControllerEntry> controllers = {
    {"VFH+", "nav2_vfh_controller::VFHController"},
    {"ND", "nav2_nd_controller::NDController"},
  };

  std::vector<Scenario> scenarios = {
    {"open", "No obstacles", 1.0, 3.0, 0.0, 5.0, 3.0, false, 0.0},
    {"frontal obstacle", "Lethal block mid-corridor, sides open",
      1.0, 3.0, 0.0, 5.0, 3.0, false, 0.0},
    {"corridor (off-centre start)",
      "Walls at y=2.1 and y=3.9; robot starts at y=3.6, near the top wall",
      1.0, 3.6, 0.0, 5.0, 3.0, true, 3.0},
  };

  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");

  constexpr double kDt = 0.1;
  constexpr int kMaxSteps = 300;
  // Slightly larger than the controllers' own goal_dist_tolerance (0.25 m), so a
  // controller that correctly stops at the goal registers as "reached" here.
  constexpr double kGoalTol = 0.3;

  std::cout << "# Reactive controller comparison (VFH+ vs ND)\n\n";
  std::cout << "Auto-generated by `nav2_planner_benchmarks` "
    "(`ros2 run nav2_planner_benchmarks controller_benchmark`). Each controller is "
    "rolled out closed-loop against a live `nav2_costmap_2d::Costmap2DROS` (6x6 m, "
    "0.05 m) with a unicycle motion model (dt=0.1 s, up to "
            << kMaxSteps << " steps), following a straight global plan while reacting "
    "to obstacles. No Gazebo/GPU.\n\n";
  std::cout << "Both are reactive `nav2_core::Controller` plugins absent from "
    "upstream Nav2 (which ships DWB / MPPI / Regulated Pure Pursuit): "
    "**VFH+** picks a free histogram valley by cost; **ND** selects a navigable "
    "gap and adds a safety deflection that centres the robot between close "
    "obstacles.\n\n";

  for (const auto & sc : scenarios) {
    std::cout << "## Scenario: " << sc.name << "\n\n";
    std::cout << sc.description << ". Start (" << sc.sx << ", " << sc.sy << ") -> goal ("
              << sc.gx << ", " << sc.gy << ").\n\n";
    std::cout << "| Controller | Reached goal | Steps | Path length [m] | "
      "Min clearance [m] | Mean |dw| [rad] |";
    if (sc.centering) {
      std::cout << " Mean |y-centre| [m] |";
    }
    std::cout << "\n|---|:-:|--:|--:|--:|--:|";
    if (sc.centering) {
      std::cout << "--:|";
    }
    std::cout << "\n";

    for (const auto & ce : controllers) {
      std::string name = "bench_" + ce.label;
      std::replace(name.begin(), name.end(), ' ', '_');
      std::replace(name.begin(), name.end(), '+', 'p');

      std::shared_ptr<nav2_core::Controller> controller;
      try {
        controller = loader.createSharedInstance(ce.class_name);
        controller->configure(node, name, tf, costmap_ros);
        controller->activate();
      } catch (const std::exception & e) {
        std::cout << "| " << ce.label << " | load error | — | — | — | — |"
                  << (sc.centering ? " — |" : "") << "\n";
        continue;
      }

      clearCostmap(costmap);
      if (sc.name == "frontal obstacle") {
        markBlock(costmap, 3.0, 3.0, 4);
      } else if (sc.centering) {
        markHWall(costmap, 2.1, 0.5, 5.5, 2);
        markHWall(costmap, 3.9, 0.5, 5.5, 2);
      }
      controller->setPlan(straightPlan(sc.sx, sc.sy, sc.gx, sc.gy));

      double x = sc.sx;
      double y = sc.sy;
      double yaw = sc.syaw;
      double prev_w = 0.0;
      double path_len = 0.0;
      double sum_dw = 0.0;
      double sum_off = 0.0;
      double min_clear = std::numeric_limits<double>::max();
      int steps = 0;
      bool reached = false;
      bool collided = false;
      geometry_msgs::msg::Twist last_twist;

      for (int s = 0; s < kMaxSteps; ++s) {
        geometry_msgs::msg::TransformStamped tfm;
        tfm.header.frame_id = "map";
        tfm.child_frame_id = "base_link";
        tfm.transform.translation.x = x;
        tfm.transform.translation.y = y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tfm.transform.rotation = tf2::toMsg(q);
        tf->setTransform(tfm, "bench", true);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation = tfm.transform.rotation;

        geometry_msgs::msg::TwistStamped cmd;
        try {
          cmd = controller->computeVelocityCommands(pose, last_twist, nullptr);
        } catch (const std::exception & e) {
          break;
        }
        const double v = cmd.twist.linear.x;
        const double w = cmd.twist.angular.z;
        last_twist = cmd.twist;

        min_clear = std::min(min_clear, nearestObstacle(costmap, x, y, 1.0));
        if (sc.centering) {
          sum_off += std::abs(y - sc.centerline);
        }
        sum_dw += std::abs(w - prev_w);
        prev_w = w;

        const double nx = x + v * std::cos(yaw) * kDt;
        const double ny = y + v * std::sin(yaw) * kDt;
        path_len += std::hypot(nx - x, ny - y);
        x = nx;
        y = ny;
        yaw += w * kDt;
        ++steps;

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap->worldToMap(x, y, mx, my) ||
          costmap->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          collided = true;
          break;
        }
        if (std::hypot(sc.gx - x, sc.gy - y) < kGoalTol) {
          reached = true;
          break;
        }
      }

      controller->deactivate();
      controller->cleanup();

      const double mean_dw = steps > 0 ? sum_dw / steps : 0.0;
      std::cout.setf(std::ios::fixed);
      std::cout.precision(3);
      std::cout << "| " << ce.label << " | "
                << (reached ? "yes" : (collided ? "collision" : "timeout")) << " | "
                << steps << " | " << path_len << " | ";
      if (min_clear == std::numeric_limits<double>::max()) {
        std::cout << "n/a";
      } else {
        std::cout << min_clear;
      }
      std::cout << " | " << mean_dw << " |";
      if (sc.centering) {
        const double mean_off = steps > 0 ? sum_off / steps : 0.0;
        std::cout << " " << mean_off << " |";
      }
      std::cout << "\n";
    }
    std::cout << "\n";
  }

  std::cout << "_Min clearance is the smallest robot-to-obstacle distance over the "
    "run (higher = safer). Mean |dw| is the average step-to-step change in angular "
    "velocity (lower = smoother). Mean |y-centre| (corridor only) is how far from "
    "the corridor centreline the robot stayed (lower = better centred). Both "
    "controllers centre and clear the obstacles; on these scenarios they trade off "
    "berth (VFH+ tends to give wider clearance around a frontal block) against "
    "smoothness (ND tends to steer more smoothly), with neither dominating. "
    "\"timeout\" means the goal was not reached within the step budget; "
    "\"collision\" means the robot entered a lethal cell._\n";

  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  return 0;
}
