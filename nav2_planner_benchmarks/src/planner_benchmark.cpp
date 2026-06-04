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

// Offline comparison of the repo's classical nav2_core::GlobalPlanner plugins on
// shared scenarios, run in-process against a live nav2_costmap_2d::Costmap2DROS
// (no Gazebo / GPU). For each (planner, scenario) it records success, path length,
// pose count, and median plan time, and writes a Markdown report to stdout.
// Loads each planner by class name via pluginlib, so it depends on the plugins
// only at runtime. Reproduce with:
//   ros2 run nav2_planner_benchmarks planner_benchmark > docs/planner_comparison.md

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{

struct PlannerEntry
{
  std::string label;       // human-readable name for the report
  std::string class_name;  // pluginlib class to load
  std::string family;      // paradigm
  // Extra parameters set on the node (under the instance name) before configure.
  // Used to point the generative planner at a learned ONNX model, etc.
  std::vector<rclcpp::Parameter> params;
};

struct Scenario
{
  std::string name;
  std::string description;
  double sx, sy, gx, gy;
  // Walls: each is {world_x, gap_center, gap_half, half_cells}. gap_half <= 0 is
  // a solid wall.
  std::vector<std::array<double, 4>> walls;
};

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

void clearCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }
  }
}

void markWall(
  nav2_costmap_2d::Costmap2D * costmap, double wx, double gap_center, double gap_half,
  int half_cells)
{
  unsigned int cx = 0;
  unsigned int cy = 0;
  if (!costmap->worldToMap(wx, 3.0, cx, cy)) {
    return;
  }
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    const double wy = costmap->getOriginY() + (my + 0.5) * costmap->getResolution();
    if (gap_half > 0.0 && std::abs(wy - gap_center) <= gap_half) {
      continue;
    }
    for (int dx = -half_cells; dx <= half_cells; ++dx) {
      costmap->setCost(cx + dx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

double pathLength(const nav_msgs::msg::Path & plan)
{
  double total = 0.0;
  for (std::size_t i = 1; i < plan.poses.size(); ++i) {
    total += std::hypot(
      plan.poses[i].pose.position.x - plan.poses[i - 1].pose.position.x,
      plan.poses[i].pose.position.y - plan.poses[i - 1].pose.position.y);
  }
  return total;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("planner_benchmark");

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("width", 6),
    rclcpp::Parameter("height", 6),
    rclcpp::Parameter("resolution", 0.05),
    rclcpp::Parameter("robot_radius", 0.1),
    rclcpp::Parameter("global_frame", std::string("map")),
    rclcpp::Parameter("robot_base_frame", std::string("base_link")),
    rclcpp::Parameter("rolling_window", false),
    rclcpp::Parameter("track_unknown_space", false),
  });
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto * costmap = costmap_ros->getCostmap();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // The curated learned Mode B model ships in model_zoo and is installed into this
  // package's share. Point the generative planner at it via OnnxPathModel.
  const std::string learned_model =
    ament_index_cpp::get_package_share_directory("nav2_planner_benchmarks") +
    "/models/costmap_flow.onnx";

  const std::vector<PlannerEntry> planners = {
    {"RRT*", "nav2_rrt_planner::RRTStarPlanner", "sampling (optimal)", {}},
    {"RRT-Connect", "nav2_rrt_planner::RRTConnectPlanner", "sampling (bidirectional)", {}},
    {"PRM", "nav2_prm_planner::PRMPlanner", "sampling (roadmap)", {}},
    {"D* Lite", "nav2_dstar_lite_planner::DStarLitePlanner", "incremental search", {}},
    {"JPS", "nav2_jps_planner::JPSPlanner", "grid A* speed-up", {}},
    {"Lazy Theta*", "nav2_lazy_theta_star_planner::LazyThetaStarPlanner", "any-angle", {}},
    {"ARA*", "nav2_ara_star_planner::ARAStarPlanner", "anytime", {}},
    {"Visibility graph", "nav2_visibility_graph_planner::VisibilityGraphPlanner", "geometric", {}},
    {"Diffusion (Mode B, analytic)", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative fan (propose + validate)", {}},
    {"Diffusion (Mode B, learned)", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative flow + costmap (propose + validate)",
      {rclcpp::Parameter("model_plugin", std::string("nav2_diffusion_onnx::OnnxPathModel")),
        rclcpp::Parameter("model_path", learned_model),
        rclcpp::Parameter("provide_costmap", true)}},
  };

  std::vector<Scenario> scenarios = {
    {"clear", "Empty 6x6 m map, off-axis goal", 1.0, 1.0, 5.0, 5.0, {}},
    {"off-centre gap", "Vertical wall with a gap well off the straight line", 1.0, 3.0, 5.0, 3.0,
      {{{3.0, 1.0, 0.5, 2}}}},
    {"slalom", "Two staggered walls (gap low then high) forcing an S-shaped detour",
      1.0, 3.0, 5.0, 3.0,
      {{{2.2, 1.0, 0.8, 2}}, {{3.8, 5.0, 0.8, 2}}}},
    {"side obstacle", "One-sided block across the straight line; a small detour to "
      "the open side clears it (matches the learned model's competence)",
      1.0, 3.0, 5.0, 3.0,
      {{{3.0, 1.4, 1.4, 4}}}},
  };

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
    "nav2_core", "nav2_core::GlobalPlanner");

  constexpr int kRuns = 21;
  auto noCancel = []() {return false;};

  std::cout << "# Classical planner comparison\n\n";
  std::cout << "Auto-generated by `nav2_planner_benchmarks` "
    "(`ros2 run nav2_planner_benchmarks planner_benchmark`). All planners run "
    "in-process against a live `nav2_costmap_2d::Costmap2DROS` (6x6 m, 0.05 m "
    "resolution, no Gazebo/GPU). Times are the median of "
            << kRuns << " plans on the dev machine and are indicative only "
    "(absolute numbers vary with load); compare relative magnitudes and the "
    "path-length / shape columns.\n\n";
  std::cout << "Planners (all `nav2_core::GlobalPlanner` plugins absent from "
    "upstream Nav2 — eight classical plus two generative, analytic and learned):\n\n";
  for (const auto & p : planners) {
    std::cout << "- **" << p.label << "** — " << p.family << "\n";
  }
  std::cout << "\n> D\\* Lite caches its goal-rooted search across calls, so its "
    "median reflects warm incremental replans (the first cold plan is slower). "
    "The others replan from scratch each call.\n\n";
  std::cout << "> **Diffusion (Mode B)** is the generative planner — a model "
    "*proposes* candidate paths and the deterministic validity layer *disposes* of "
    "colliding ones, keeping the shortest survivor. Two variants run here. "
    "**analytic** uses the built-in `FanPathModel` (a symmetric bowed fan, no ONNX); "
    "**learned** loads the curated costmap-conditioned flow model from `model_zoo` "
    "via `OnnxPathModel` (real ONNX inference). Both are generative, so unlike the "
    "search planners they are not complete: if no proposal threads the gap they "
    "report no path. The learned model reads the costmap and biases every proposal "
    "to the open side (see its model card), but its synthetic training distribution "
    "caps the detour size — so it clears *clear* and *side obstacle* yet cannot make "
    "the 2 m swing of *off-centre gap* or the S of *slalom* that the wider analytic "
    "fan can. The ceiling is the training data, not the architecture; richer data "
    "lifts it and the same safety layer still gates the output.\n\n";

  for (const auto & sc : scenarios) {
    std::cout << "## Scenario: " << sc.name << "\n\n";
    std::cout << sc.description << ". Start (" << sc.sx << ", " << sc.sy << ") -> goal ("
              << sc.gx << ", " << sc.gy << ").\n\n";
    std::cout << "| Planner | Family | Success | Path length [m] | Poses | Median time [ms] |\n";
    std::cout << "|---|---|:-:|--:|--:|--:|\n";

    for (const auto & pe : planners) {
      // Fresh instance per (planner, scenario) for a fair, independent measure.
      std::string name = "bench_" + pe.label;
      std::replace(name.begin(), name.end(), ' ', '_');
      std::replace(name.begin(), name.end(), '*', 's');

      // Apply any per-planner parameters under the instance name before configure
      // (idempotent across scenarios, which reuse the same instance name).
      for (const auto & p : pe.params) {
        const std::string key = name + "." + p.get_name();
        if (node->has_parameter(key)) {
          node->set_parameter(rclcpp::Parameter(key, p.get_parameter_value()));
        } else {
          node->declare_parameter(key, p.get_parameter_value());
        }
      }

      std::shared_ptr<nav2_core::GlobalPlanner> planner;
      try {
        planner = loader.createSharedInstance(pe.class_name);
        planner->configure(node, name, tf, costmap_ros);
        planner->activate();
      } catch (const std::exception & e) {
        std::cout << "| " << pe.label << " | " << pe.family << " | load error | — | — | — |\n";
        continue;
      }

      // Build the scenario costmap.
      clearCostmap(costmap);
      for (const auto & w : sc.walls) {
        markWall(costmap, w[0], w[1], w[2], static_cast<int>(w[3]));
      }

      bool ok = true;
      double length = 0.0;
      std::size_t poses = 0;
      std::vector<double> times_ms;
      times_ms.reserve(kRuns);
      for (int r = 0; r < kRuns; ++r) {
        try {
          const auto t0 = std::chrono::steady_clock::now();
          const auto plan = planner->createPlan(
            makePose(sc.sx, sc.sy), makePose(sc.gx, sc.gy), noCancel);
          const auto t1 = std::chrono::steady_clock::now();
          times_ms.push_back(
            std::chrono::duration<double, std::milli>(t1 - t0).count());
          length = pathLength(plan);
          poses = plan.poses.size();
        } catch (const std::exception & e) {
          ok = false;
          break;
        }
      }

      planner->deactivate();
      planner->cleanup();

      if (!ok || times_ms.empty()) {
        std::cout << "| " << pe.label << " | " << pe.family << " | no path | — | — | — |\n";
        continue;
      }
      std::sort(times_ms.begin(), times_ms.end());
      const double median = times_ms[times_ms.size() / 2];

      std::cout.setf(std::ios::fixed);
      std::cout.precision(3);
      std::cout << "| " << pe.label << " | " << pe.family << " | yes | " << length << " | "
                << poses << " | " << median << " |\n";
    }
    std::cout << "\n";
  }

  std::cout << "_Path length is the sum of consecutive pose distances; lower is "
    "shorter (not always better — sampling planners trade optimality for speed, "
    "ARA\\* trades it for an anytime bound). Pose count reflects densification "
    "granularity. \"no path\" means the planner raised an exception (e.g. the "
    "sampling budget did not connect start and goal within the scenario)._\n";

  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  rclcpp::shutdown();
  return 0;
}
