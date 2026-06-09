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

// Trace exporter for the Nav2PlannerBattle web game (tools/nav2_planner_battle).
// Runs the REAL planner / controller plugins (same as the benchmarks) and dumps
// per-step traces to JSON so the browser viewer can replay an honest head-to-head:
//   * Mode A (local controllers): closed-loop unicycle rollout on shared scenarios,
//     recording every (x, y, yaw) pose + outcome -> an arena RACE.
//   * Mode B (global planners): createPlan on shared scenarios, recording the full
//     path + median planning time -> a path DUEL.
// This deliberately mirrors controller_benchmark.cpp / planner_benchmark.cpp; it
// reports the same real behaviour, only as machine-readable JSON instead of
// Markdown. Reproduce with:
//   ros2 run nav2_planner_benchmarks battle_trace > tools/nav2_planner_battle/battle_data.json
// Custom ONNX fighters: docs/custom_model_battle.md

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_planner_benchmarks/micro_mouse_maze.hpp"

namespace
{
using nav2_planner_benchmarks::isMicroMouseScenario;
using nav2_planner_benchmarks::markMicroMouseScenario;
using nav2_planner_benchmarks::microMouseLayouts;
using nav2_planner_benchmarks::microMouseObstacleRects;

constexpr double kRes = 0.05;        // costmap resolution [m]
constexpr double kArena = 6.0;       // 6 x 6 m

struct Rect { double x, y, w, h; };  // lower-left + size, world metres

struct Entry
{
  std::string label;
  std::string class_name;
  std::string family;
  std::vector<rclcpp::Parameter> params;
};

struct CustomEntry
{
  std::string label;
  std::string onnx_path;
  std::string family;
  double min_turn_radius = -1.0;
  bool is_planner = false;
};

enum class ExportMode {Both, AOnly, BOnly};

struct BattleCli
{
  std::vector<CustomEntry> custom;
  bool custom_only = false;
  ExportMode export_mode = ExportMode::Both;
};

bool isAbsPath(const std::string & path)
{
  return !path.empty() && (path[0] == '/' || (path.size() > 1 && path[1] == ':'));
}

void printBattleHelp()
{
  std::cerr <<
    "battle_trace — export Nav2 Planner Battle JSON from real Nav2 plugins.\n"
    "\n"
    "Optional flags (stripped before rclcpp init):\n"
    "  --custom-controller LABEL ONNX [FAMILY]\n"
    "      Append a Mode A DiffusionController using ONNX at ONNX (absolute or share/models/ name).\n"
    "  --custom-planner LABEL ONNX [MIN_TURN_RADIUS]\n"
    "      Append a Mode B DiffusionGlobalPlanner using ONNX (default R=0 omni if omitted).\n"
    "  --custom-only          Run only --custom-* entries (skip the default roster).\n"
    "  --mode A|B|both        Limit export to one battle mode (default: both).\n"
    "  -h, --help             Show this help.\n"
    "\n"
    "Examples:\n"
    "  ros2 run nav2_planner_benchmarks battle_trace > battle_data.json\n"
    "  ros2 run nav2_planner_benchmarks battle_trace \\\n"
    "    --mode A --custom-only --custom-controller my-flow /tmp/local.onnx > custom.json\n";
}

BattleCli parseBattleCli(int & argc, char ** argv)
{
  BattleCli cli;
  std::vector<char *> kept;
  kept.push_back(argv[0]);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printBattleHelp();
      std::exit(0);
    } else if (arg == "--custom-only") {
      cli.custom_only = true;
    } else if (arg == "--mode" && i + 1 < argc) {
      const std::string mode = argv[++i];
      if (mode == "A" || mode == "a") {
        cli.export_mode = ExportMode::AOnly;
      } else if (mode == "B" || mode == "b") {
        cli.export_mode = ExportMode::BOnly;
      } else if (mode == "both") {
        cli.export_mode = ExportMode::Both;
      } else {
        std::cerr << "battle_trace: unknown --mode " << mode << " (use A, B, or both)\n";
        std::exit(2);
      }
    } else if (arg == "--custom-controller" && i + 2 < argc) {
      CustomEntry e;
      e.label = argv[++i];
      e.onnx_path = argv[++i];
      e.is_planner = false;
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        e.family = argv[++i];
      } else {
        e.family = "custom (local)";
      }
      cli.custom.push_back(e);
    } else if (arg == "--custom-planner" && i + 2 < argc) {
      CustomEntry e;
      e.label = argv[++i];
      e.onnx_path = argv[++i];
      e.is_planner = true;
      e.family = "custom (global)";
      if (i + 1 < argc && argv[i + 1][0] != '-') {
        e.min_turn_radius = std::stod(argv[++i]);
      }
      cli.custom.push_back(e);
    } else {
      kept.push_back(argv[i]);
    }
  }
  argc = static_cast<int>(kept.size());
  for (int i = 0; i < argc; ++i) {
    argv[i] = kept[i];
  }
  return cli;
}

// --- shared costmap helpers (mirror the benchmarks) ---

void clearCostmap(nav2_costmap_2d::Costmap2D * costmap)
{
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }
  }
}

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

void markHWall(
  nav2_costmap_2d::Costmap2D * costmap, double wy, double x0, double x1, int half_cells)
{
  for (double x = x0; x <= x1; x += kRes) {
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

// Vertical wall at world x = wx with an optional gap, mirroring planner_benchmark markWall.
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

// Rect builders that match the mark* params (so the drawn geometry == the costmap).
Rect blockRect(double wx, double wy, int half_cells)
{
  const double extent = (2 * half_cells + 1) * kRes;
  return Rect{wx - extent / 2.0, wy - extent / 2.0, extent, extent};
}

Rect hwallRect(double wy, double x0, double x1, int half_cells)
{
  const double th = (2 * half_cells + 1) * kRes;
  return Rect{x0, wy - th / 2.0, x1 - x0, th};
}

std::vector<Rect> mazeDisplayRects(const std::string & scenario_name)
{
  std::vector<Rect> out;
  for (const auto & r : microMouseObstacleRects(scenario_name)) {
    out.push_back(Rect{r.x, r.y, r.w, r.h});
  }
  return out;
}

std::vector<Rect> wallRects(double wx, double gap_center, double gap_half, int half_cells)
{
  const double th = (2 * half_cells + 1) * kRes;
  const double x = wx - th / 2.0;
  std::vector<Rect> rects;
  if (gap_half <= 0.0) {
    rects.push_back(Rect{x, 0.0, th, kArena});
    return rects;
  }
  const double lo = gap_center - gap_half;
  const double hi = gap_center + gap_half;
  if (lo > 0.0) {
    rects.push_back(Rect{x, 0.0, th, lo});
  }
  if (hi < kArena) {
    rects.push_back(Rect{x, hi, th, kArena - hi});
  }
  return rects;
}

double nearestObstacle(nav2_costmap_2d::Costmap2D * costmap, double x, double y, double window)
{
  unsigned int rmx = 0;
  unsigned int rmy = 0;
  if (!costmap->worldToMap(x, y, rmx, rmy)) {
    return 0.0;
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

geometry_msgs::msg::PoseStamped makePose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  return p;
}

nav_msgs::msg::Path straightPlan(double sx, double sy, double gx, double gy)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  const double dist = std::hypot(gx - sx, gy - sy);
  const int n = std::max(2, static_cast<int>(dist / 0.1));
  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    path.poses.push_back(makePose(sx + t * (gx - sx), sy + t * (gy - sy)));
  }
  return path;
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

std::string sanitize(std::string name, const std::string & prefix)
{
  name = prefix + name;
  for (char & ch : name) {
    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_') {
      ch = '_';
    }
  }
  return name;
}

// --- tiny JSON writer (no external dependency) ---
struct Json
{
  std::ostream & os;
  int n = 3;
  void num(double v) {os << (std::round(v * 1000.0) / 1000.0);}
  void str(const std::string & s)
  {
    os << '"';
    for (char c : s) {
      if (c == '"' || c == '\\') {os << '\\';}
      os << c;
    }
    os << '"';
  }
};

struct ModeAScenario
{
  std::string name, description;
  double sx, sy, gx, gy;
  std::vector<Rect> obstacles;
  bool centering{false};
  double centerline{3.0};
};

struct ModeBScenario
{
  std::string name, description;
  double sx, sy, gx, gy;
  std::vector<std::array<double, 4>> walls;   // {wx, gap_center, gap_half, half_cells}
};

}  // namespace

int main(int argc, char ** argv)
{
  const BattleCli cli = parseBattleCli(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("battle_trace");

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("width", 6),
    rclcpp::Parameter("height", 6),
    rclcpp::Parameter("resolution", kRes),
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

  const std::string share = ament_index_cpp::get_package_share_directory("nav2_planner_benchmarks");
  const auto model = [&](const std::string & f) {return share + "/models/" + f;};
  const auto resolveOnnx = [&](const std::string & f) {
      return isAbsPath(f) ? f : model(f);
    };
  auto onnxA = [&](const std::string & f, std::vector<rclcpp::Parameter> extra) {
      std::vector<rclcpp::Parameter> p = {
        rclcpp::Parameter("model_plugin", std::string("nav2_diffusion_onnx::OnnxTrajectoryModel")),
        rclcpp::Parameter("model_path", resolveOnnx(f)),
        rclcpp::Parameter("costmap_patch_size", 32),
        rclcpp::Parameter("lookahead_distance", 1.0),
        rclcpp::Parameter("max_linear_speed", 1.5)};
      p.insert(p.end(), extra.begin(), extra.end());
      return p;
    };
  auto onnxB = [&](const std::string & f, std::vector<rclcpp::Parameter> extra) {
      std::vector<rclcpp::Parameter> p = {
        rclcpp::Parameter("model_plugin", std::string("nav2_diffusion_onnx::OnnxPathModel")),
        rclcpp::Parameter("model_path", resolveOnnx(f)),
        rclcpp::Parameter("provide_costmap", true)};
      p.insert(p.end(), extra.begin(), extra.end());
      return p;
    };

  std::vector<Entry> controllers = {
    {"VFH+", "nav2_vfh_controller::VFHController", "reactive", {}},
    {"ND", "nav2_nd_controller::NDController", "reactive", {}},
    {"learned", "nav2_diffusion_controller::DiffusionController", "generative flow",
      onnxA("diffusion_local_costmap_flow.onnx", {})},
    {"transformer", "nav2_diffusion_controller::DiffusionController", "generative transformer",
      onnxA("diffusion_local_costmap_transformer.onnx", {})},
    {"recurrent", "nav2_diffusion_controller::DiffusionController", "generative GRU",
      onnxA("diffusion_local_costmap_recurrent.onnx", {})},
    {"threading", "nav2_diffusion_controller::DiffusionController",
      "generative (threads obstacles)",
      onnxA(
        "diffusion_local_costmap_threading.onnx",
        {rclcpp::Parameter("safety_check_points", 3),
          rclcpp::Parameter("costmap_patch_resolution", 0.08)})},
    {"hybrid", "nav2_diffusion_controller::DiffusionController", "generative + VFH+ fallback",
      onnxA(
        "diffusion_local_costmap_flow.onnx",
        {rclcpp::Parameter(
            "fallback_controller_plugin", std::string("nav2_vfh_controller::VFHController"))})},
  };

  std::vector<ModeAScenario> aScenarios = {
    {"open", "No obstacles", 1.0, 3.0, 5.0, 3.0, {}},
    {"frontal", "Lethal block dead ahead, sides open", 1.0, 3.0, 5.0, 3.0,
      {blockRect(3.0, 3.0, 4)}},
    {"side", "Off-centre block overlapping the straight path", 1.0, 3.0, 5.0, 3.0,
      {blockRect(3.0, 3.3, 6)}},
    {"corridor", "Two walls (y=2.1, y=3.9); off-centre start near the top wall",
      1.0, 3.6, 5.0, 3.0, {hwallRect(2.1, 0.5, 5.5, 2), hwallRect(3.9, 0.5, 5.5, 2)},
      true, 3.0},
  };
  for (const auto & maze : microMouseLayouts()) {
    aScenarios.push_back(
      {maze.name, maze.description, maze.start_x, maze.start_y, maze.goal_x, maze.goal_y,
        mazeDisplayRects(maze.name), false, 3.0});
  }

  std::vector<Entry> planners = {
    {"RRT*", "nav2_rrt_planner::RRTStarPlanner", "sampling (optimal)", {}},
    {"RRT-Connect", "nav2_rrt_planner::RRTConnectPlanner", "sampling (bidirectional)", {}},
    {"PRM", "nav2_prm_planner::PRMPlanner", "sampling (roadmap)", {}},
    {"D* Lite", "nav2_dstar_lite_planner::DStarLitePlanner", "incremental search", {}},
    {"JPS", "nav2_jps_planner::JPSPlanner", "grid A* speed-up", {}},
    {"Lazy Theta*", "nav2_lazy_theta_star_planner::LazyThetaStarPlanner", "any-angle", {}},
    {"ARA*", "nav2_ara_star_planner::ARAStarPlanner", "anytime", {}},
    {"Visibility graph", "nav2_visibility_graph_planner::VisibilityGraphPlanner", "geometric", {}},
    {"Diffusion analytic", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative fan", {}},
    {"Diffusion learned", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative flow", onnxB("costmap_flow.onnx", {})},
    {"Diffusion transformer", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative transformer", onnxB("diffusion_global_costmap_transformer.onnx", {})},
    {"Diffusion attnseq", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative cross-attn (8/8 courses)", onnxB("diffusion_global_costmap_attnseq.onnx", {})},
    {"Diffusion hybrid", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "generative + JPS fallback",
      onnxB(
        "costmap_flow.onnx",
        {rclcpp::Parameter("fallback_planner_plugin",
          std::string("nav2_jps_planner::JPSPlanner"))})},
    {"Diffusion omni", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "kinematics R=0 (omni)",
      onnxB(
        "diffusion_global_costmap_kinematics.onnx",
        {rclcpp::Parameter("min_turn_radius", 0.0)})},
    {"Diffusion diff", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "kinematics R=0.3 (diff-drive)",
      onnxB(
        "diffusion_global_costmap_kinematics.onnx",
        {rclcpp::Parameter("min_turn_radius", 0.3)})},
    {"Diffusion Ackermann", "nav2_diffusion_global_planner::DiffusionGlobalPlanner",
      "kinematics R=1.5 (Ackermann)",
      onnxB(
        "diffusion_global_costmap_kinematics.onnx",
        {rclcpp::Parameter("min_turn_radius", 1.5)})},
  };

  const bool custom_controllers = std::any_of(
    cli.custom.begin(), cli.custom.end(), [](const CustomEntry & e) {return !e.is_planner;});
  const bool custom_planners = std::any_of(
    cli.custom.begin(), cli.custom.end(), [](const CustomEntry & e) {return e.is_planner;});
  if (cli.custom_only) {
    if (custom_controllers) {
      controllers.clear();
    }
    if (custom_planners) {
      planners.clear();
    }
  }
  for (const auto & c : cli.custom) {
    if (c.is_planner) {
      std::vector<rclcpp::Parameter> extra;
      if (c.min_turn_radius >= 0.0) {
        extra.push_back(rclcpp::Parameter("min_turn_radius", c.min_turn_radius));
      }
      planners.push_back(
        {c.label, "nav2_diffusion_global_planner::DiffusionGlobalPlanner", c.family,
          onnxB(c.onnx_path, extra)});
    } else {
      controllers.push_back(
        {c.label, "nav2_diffusion_controller::DiffusionController", c.family,
          onnxA(c.onnx_path, {})});
    }
  }

  std::vector<ModeBScenario> bScenarios = {
    {"clear", "Empty map, off-axis goal", 1.0, 1.0, 5.0, 5.0, {}},
    {"centred gap", "Wall with a gap dead ahead", 1.0, 3.0, 5.0, 3.0, {{{3.0, 3.0, 0.5, 2}}}},
    {"narrow gap", "Tight gap centred on the line", 1.0, 3.0, 5.0, 3.0, {{{3.0, 3.0, 0.3, 2}}}},
    {"off-centre gap", "Gap well off the straight line", 1.0, 3.0, 5.0, 3.0,
      {{{3.0, 1.0, 0.5, 2}}}},
    {"far off-centre gap", "Off-axis gap pushed toward the goal", 1.0, 3.0, 5.0, 3.0,
      {{{4.0, 1.0, 0.5, 2}}}},
    {"double gate", "Two in-line gaps in series", 1.0, 3.0, 5.0, 3.0,
      {{{2.2, 3.0, 0.6, 2}}, {{3.8, 3.0, 0.6, 2}}}},
    {"slalom", "Two staggered gaps forcing an S-detour", 1.0, 3.0, 5.0, 3.0,
      {{{2.2, 1.0, 0.8, 2}}, {{3.8, 5.0, 0.8, 2}}}},
    {"side obstacle", "One-sided block across the line", 1.0, 3.0, 5.0, 3.0,
      {{{3.0, 1.4, 1.4, 4}}}},
  };
  for (const auto & maze : microMouseLayouts()) {
    bScenarios.push_back(
      {maze.name, maze.description, maze.start_x, maze.start_y, maze.goal_x, maze.goal_y, {}});
  }

  pluginlib::ClassLoader<nav2_core::Controller> cloader(
    "nav2_core", "nav2_core::Controller");
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> ploader(
    "nav2_core", "nav2_core::GlobalPlanner");

  constexpr double kDt = 0.1;
  constexpr int kMaxSteps = 300;
  constexpr double kGoalTol = 0.3;
  constexpr int kRuns = 9;
  auto noCancel = []() {return false;};

  Json j{std::cout};
  std::cout.setf(std::ios::fixed);
  std::cout.precision(3);

  const bool export_a = cli.export_mode != ExportMode::BOnly;
  const bool export_b = cli.export_mode != ExportMode::AOnly;
  if (export_a && controllers.empty()) {
    std::cerr << "battle_trace: no Mode A fighters to run\n";
    return 2;
  }
  if (export_b && planners.empty()) {
    std::cerr << "battle_trace: no Mode B fighters to run\n";
    return 2;
  }

  std::cout << "{\n";
  std::cout << "\"arena\":{\"w\":" << kArena << ",\"h\":" << kArena << "},\n";

  // ---------- Mode A: arena race ----------
  if (export_a) {
  std::cout << "\"modeA\":{\"title\":\"Mode A — local controller arena race\",\"dt\":" << kDt
            << ",\"scenarios\":[\n";
  for (std::size_t si = 0; si < aScenarios.size(); ++si) {
    const auto & sc = aScenarios[si];
    std::cout << "{\"name\":"; j.str(sc.name);
    std::cout << ",\"description\":"; j.str(sc.description);
    std::cout << ",\"start\":[" << sc.sx << "," << sc.sy << "],\"goal\":[" << sc.gx << ","
              << sc.gy << "],\"obstacles\":[";
    for (std::size_t oi = 0; oi < sc.obstacles.size(); ++oi) {
      const auto & r = sc.obstacles[oi];
      std::cout << "{\"x\":" << r.x << ",\"y\":" << r.y << ",\"w\":" << r.w << ",\"h\":" << r.h
                << "}" << (oi + 1 < sc.obstacles.size() ? "," : "");
    }
    std::cout << "],\"metrics\":{";
    if (sc.centering) {
      std::cout << "\"centering\":true,\"centerline\":" << sc.centerline;
    } else {
      std::cout << "\"centering\":false";
    }
    std::cout << "},\"fighters\":[\n";

    for (std::size_t ci = 0; ci < controllers.size(); ++ci) {
      const auto & ce = controllers[ci];
      const std::string name = sanitize(ce.label, "battleA_");
      for (const auto & p : ce.params) {
        const std::string key = name + "." + p.get_name();
        if (node->has_parameter(key)) {
          node->set_parameter(rclcpp::Parameter(key, p.get_parameter_value()));
        } else {
          node->declare_parameter(key, p.get_parameter_value());
        }
      }
      std::shared_ptr<nav2_core::Controller> controller;
      std::string outcome = "reached";
      std::vector<std::array<double, 3>> trace;
      double path_len = 0.0;
      double min_clear = std::numeric_limits<double>::max();
      double sum_dw = 0.0;
      double sum_off = 0.0;
      double prev_w = 0.0;
      int steps = 0;
      try {
        controller = cloader.createSharedInstance(ce.class_name);
        controller->configure(node, name, tf, costmap_ros);
        controller->activate();

        clearCostmap(costmap);
        if (sc.name == "frontal") {
          markBlock(costmap, 3.0, 3.0, 4);
        } else if (sc.name == "side") {
          markBlock(costmap, 3.0, 3.3, 6);
        } else if (sc.name == "corridor") {
          markHWall(costmap, 2.1, 0.5, 5.5, 2);
          markHWall(costmap, 3.9, 0.5, 5.5, 2);
        } else if (isMicroMouseScenario(sc.name)) {
          markMicroMouseScenario(costmap, sc.name);
        }
        controller->setPlan(straightPlan(sc.sx, sc.sy, sc.gx, sc.gy));

        double x = sc.sx, y = sc.sy, yaw = 0.0;
        geometry_msgs::msg::Twist last_twist;
        bool reached = false, collided = false;
        trace.push_back({x, y, yaw});
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
          } catch (const std::exception &) {
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
          trace.push_back({x, y, yaw});
          ++steps;

          unsigned int mx = 0, my = 0;
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
        outcome = reached ? "reached" : (collided ? "collision" : "timeout");
      } catch (const std::exception &) {
        outcome = "error";
      }

      const double mean_dw = steps > 0 ? sum_dw / steps : 0.0;
      const double mean_center = (sc.centering && steps > 0) ? sum_off / steps : -1.0;

      std::cout << "{\"label\":"; j.str(ce.label);
      std::cout << ",\"family\":"; j.str(ce.family);
      std::cout << ",\"outcome\":"; j.str(outcome);
      std::cout << ",\"steps\":" << (trace.empty() ? 0 : static_cast<int>(trace.size()) - 1);
      std::cout << ",\"length\":" << path_len;
      std::cout << ",\"clearance\":"
                << (min_clear == std::numeric_limits<double>::max() ? 0.0 : min_clear);
      std::cout << ",\"mean_dw\":" << mean_dw;
      if (mean_center >= 0.0) {
        std::cout << ",\"centering\":" << mean_center;
      }
      std::cout << ",\"path\":[";
      for (std::size_t k = 0; k < trace.size(); ++k) {
        std::cout << "[" << trace[k][0] << "," << trace[k][1] << "," << trace[k][2] << "]"
                  << (k + 1 < trace.size() ? "," : "");
      }
      std::cout << "]}" << (ci + 1 < controllers.size() ? ",\n" : "\n");
    }
    std::cout << "]}" << (si + 1 < aScenarios.size() ? ",\n" : "\n");
  }
  std::cout << "]}";
  if (export_b) {
    std::cout << ",\n";
  } else {
    std::cout << "\n";
  }
  }

  // ---------- Mode B: path duel ----------
  if (export_b) {
  std::cout << "\"modeB\":{\"title\":\"Mode B — global planner path duel\",\"scenarios\":[\n";
  for (std::size_t si = 0; si < bScenarios.size(); ++si) {
    const auto & sc = bScenarios[si];
    std::cout << "{\"name\":"; j.str(sc.name);
    std::cout << ",\"description\":"; j.str(sc.description);
    std::cout << ",\"start\":[" << sc.sx << "," << sc.sy << "],\"goal\":[" << sc.gx << ","
              << sc.gy << "],\"obstacles\":[";
    bool first_rect = true;
    if (isMicroMouseScenario(sc.name)) {
      for (const auto & r : microMouseObstacleRects(sc.name)) {
        std::cout << (first_rect ? "" : ",") << "{\"x\":" << r.x << ",\"y\":" << r.y
                  << ",\"w\":" << r.w << ",\"h\":" << r.h << "}";
        first_rect = false;
      }
    } else {
      for (const auto & w : sc.walls) {
        for (const auto & r : wallRects(w[0], w[1], w[2], static_cast<int>(w[3]))) {
          std::cout << (first_rect ? "" : ",") << "{\"x\":" << r.x << ",\"y\":" << r.y
                    << ",\"w\":" << r.w << ",\"h\":" << r.h << "}";
          first_rect = false;
        }
      }
    }
    std::cout << "],\"fighters\":[\n";

    for (std::size_t pi = 0; pi < planners.size(); ++pi) {
      const auto & pe = planners[pi];
      std::string name = sanitize(pe.label, "battleB_");
      std::replace(name.begin(), name.end(), '*', 's');
      for (const auto & p : pe.params) {
        const std::string key = name + "." + p.get_name();
        if (node->has_parameter(key)) {
          node->set_parameter(rclcpp::Parameter(key, p.get_parameter_value()));
        } else {
          node->declare_parameter(key, p.get_parameter_value());
        }
      }
      std::shared_ptr<nav2_core::GlobalPlanner> planner;
      bool ok = true;
      double length = 0.0;
      nav_msgs::msg::Path best_plan;
      std::vector<double> times_ms;
      try {
        planner = ploader.createSharedInstance(pe.class_name);
        planner->configure(node, name, tf, costmap_ros);
        planner->activate();
        clearCostmap(costmap);
        if (isMicroMouseScenario(sc.name)) {
          markMicroMouseScenario(costmap, sc.name);
        } else {
          for (const auto & w : sc.walls) {
            markWall(costmap, w[0], w[1], w[2], static_cast<int>(w[3]));
          }
        }
        for (int r = 0; r < kRuns; ++r) {
          const auto t0 = std::chrono::steady_clock::now();
          const auto plan = planner->createPlan(
            makePose(sc.sx, sc.sy), makePose(sc.gx, sc.gy), noCancel);
          const auto t1 = std::chrono::steady_clock::now();
          times_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
          best_plan = plan;
          length = pathLength(plan);
        }
        planner->deactivate();
        planner->cleanup();
        if (times_ms.empty() || best_plan.poses.empty()) {ok = false;}
      } catch (const std::exception &) {
        ok = false;
      }
      double median = 0.0;
      if (!times_ms.empty()) {
        std::sort(times_ms.begin(), times_ms.end());
        median = times_ms[times_ms.size() / 2];
      }

      std::cout << "{\"label\":"; j.str(pe.label);
      std::cout << ",\"family\":"; j.str(pe.family);
      std::cout << ",\"success\":" << (ok ? "true" : "false");
      std::cout << ",\"length\":" << length;
      std::cout << ",\"poses\":" << best_plan.poses.size();
      std::cout << ",\"time_ms\":" << median;
      std::cout << ",\"path\":[";
      if (ok) {
        for (std::size_t k = 0; k < best_plan.poses.size(); ++k) {
          std::cout << "[" << best_plan.poses[k].pose.position.x << ","
                    << best_plan.poses[k].pose.position.y << "]"
                    << (k + 1 < best_plan.poses.size() ? "," : "");
        }
      }
      std::cout << "]}" << (pi + 1 < planners.size() ? ",\n" : "\n");
    }
    std::cout << "]}" << (si + 1 < bScenarios.size() ? ",\n" : "\n");
  }
  std::cout << "]}\n";
  }

  std::cout << "}\n";

  rclcpp::shutdown();
  return 0;
}
