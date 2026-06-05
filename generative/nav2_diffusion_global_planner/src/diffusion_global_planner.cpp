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

#include "nav2_diffusion_global_planner/diffusion_global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_diffusion_global_planner
{

void DiffusionGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();

  using nav2_util::declare_parameter_if_not_declared;
  declare_parameter_if_not_declared(node, name_ + ".num_candidates", rclcpp::ParameterValue(11));
  declare_parameter_if_not_declared(node, name_ + ".num_points", rclcpp::ParameterValue(40));
  declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name_ + ".max_bow_fraction", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, name_ + ".provide_costmap", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, name_ + ".model_plugin", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, name_ + ".model_path", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, name_ + ".fallback_planner_plugin", rclcpp::ParameterValue(std::string("")));
  declare_parameter_if_not_declared(
    node, name_ + ".hybrid_mode", rclcpp::ParameterValue(std::string("fallback")));
  declare_parameter_if_not_declared(
    node, name_ + ".guidance_strength", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, name_ + ".guidance_radius", rclcpp::ParameterValue(0.3));

  node->get_parameter(name_ + ".num_candidates", num_candidates_);
  node->get_parameter(name_ + ".num_points", num_points_);
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".max_bow_fraction", max_bow_fraction_);
  node->get_parameter(name_ + ".provide_costmap", provide_costmap_);
  node->get_parameter(name_ + ".model_plugin", model_plugin_);
  node->get_parameter(name_ + ".model_path", model_path_);
  node->get_parameter(name_ + ".fallback_planner_plugin", fallback_planner_plugin_);
  node->get_parameter(name_ + ".hybrid_mode", hybrid_mode_);
  node->get_parameter(name_ + ".guidance_strength", guidance_strength_);
  node->get_parameter(name_ + ".guidance_radius", guidance_radius_);

  if (model_plugin_.empty()) {
    model_ = std::make_shared<nav2_diffusion_core::FanPathModel>(max_bow_fraction_);
  } else {
    model_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_diffusion_core::PathModel>>(
      "nav2_diffusion_core", "nav2_diffusion_core::PathModel");
    model_ = model_loader_->createSharedInstance(model_plugin_);
    model_->configure(model_path_);
  }

  // Optional classical fallback (a complete search) for when no generative
  // candidate is valid — the "search disposes" half of the hybrid.
  if (!fallback_planner_plugin_.empty()) {
    try {
      fallback_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
        "nav2_core", "nav2_core::GlobalPlanner");
      fallback_planner_ = fallback_loader_->createSharedInstance(fallback_planner_plugin_);
      fallback_planner_->configure(parent, name_ + ".fallback", tf, costmap_ros);
      RCLCPP_INFO(
        logger_, "Loaded fallback planner '%s'", fallback_planner_plugin_.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "Failed to load fallback planner '%s': %s; will throw on no path instead",
        fallback_planner_plugin_.c_str(), ex.what());
      fallback_planner_.reset();
      fallback_loader_.reset();
    }
  }

  RCLCPP_INFO(
    logger_, "DiffusionGlobalPlanner '%s' configured: model='%s', %d candidates, %d points",
    name_.c_str(), model_->name().c_str(), num_candidates_, num_points_);
}

void DiffusionGlobalPlanner::cleanup()
{
  if (fallback_planner_) {
    fallback_planner_->cleanup();
    fallback_planner_.reset();
  }
  fallback_loader_.reset();
  model_.reset();
  model_loader_.reset();
}

void DiffusionGlobalPlanner::activate()
{
  if (fallback_planner_) {
    fallback_planner_->activate();
  }
}

void DiffusionGlobalPlanner::deactivate()
{
  if (fallback_planner_) {
    fallback_planner_->deactivate();
  }
}

void DiffusionGlobalPlanner::fillCostmap(nav2_diffusion_core::PathContext & ctx) const
{
  const unsigned int sx = costmap_->getSizeInCellsX();
  const unsigned int sy = costmap_->getSizeInCellsY();
  ctx.costmap_size_x = sx;
  ctx.costmap_size_y = sy;
  ctx.costmap_resolution = costmap_->getResolution();
  ctx.costmap_origin_x = costmap_->getOriginX();
  ctx.costmap_origin_y = costmap_->getOriginY();
  ctx.costmap.resize(static_cast<std::size_t>(sx) * sy);
  for (unsigned int my = 0; my < sy; ++my) {
    for (unsigned int mx = 0; mx < sx; ++mx) {
      const unsigned char cost = costmap_->getCost(mx, my);
      float value;
      if (cost == nav2_costmap_2d::NO_INFORMATION) {
        value = 0.0f;  // unknown -> free for the model; validity layer handles it
      } else if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        value = 1.0f;
      } else {
        value = static_cast<float>(cost) /
          static_cast<float>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
      }
      ctx.costmap[static_cast<std::size_t>(my) * sx + mx] = value;
    }
  }
}

bool DiffusionGlobalPlanner::isCellTraversable(double wx, double wy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    return false;  // outside the costmap bounds
  }
  const unsigned char cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
  {
    return false;
  }
  if (cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_) {
    return false;
  }
  return true;
}

bool DiffusionGlobalPlanner::isPathValid(const nav2_diffusion_core::PathCandidate & path) const
{
  const double step = std::max(interpolation_resolution_, 1e-3);
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const auto & a = path.points[i - 1];
    const auto & b = path.points[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    const int samples = std::max(1, static_cast<int>(std::ceil(seg / step)));
    for (int s = 0; s <= samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(samples);
      if (!isCellTraversable(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))) {
        return false;
      }
    }
  }
  return true;
}

nav_msgs::msg::Path DiffusionGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  auto node = node_.lock();
  plan.header.stamp = node ? node->now() : rclcpp::Clock().now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerTFError(
            "DiffusionGlobalPlanner expects start/goal in the " + global_frame_ + " frame");
  }
  if (!isCellTraversable(start.pose.position.x, start.pose.position.y)) {
    throw nav2_core::StartOccupied("Start pose is in a lethal / out-of-bounds cell");
  }
  if (!isCellTraversable(goal.pose.position.x, goal.pose.position.y)) {
    throw nav2_core::GoalOccupied("Goal pose is in a lethal / out-of-bounds cell");
  }

  // Propose: ask the generative model for K candidate paths.
  nav2_diffusion_core::PathContext ctx;
  ctx.start_x = start.pose.position.x;
  ctx.start_y = start.pose.position.y;
  ctx.goal_x = goal.pose.position.x;
  ctx.goal_y = goal.pose.position.y;
  ctx.num_candidates = num_candidates_;
  ctx.num_points = num_points_;
  // Hand the (normalized) global costmap to costmap-conditioned models; analytic
  // models ignore it. The deterministic validity layer below remains the
  // authority on collisions regardless.
  if (provide_costmap_) {
    fillCostmap(ctx);
  }
  const auto candidates = model_->generate(ctx);

  // Tightly-coupled hybrid: run a complete A* whose costs are discounted near the
  // valid proposals, so the learned model shapes the route and the search keeps
  // completeness. (The fallback mode below only invokes search on total failure.)
  if (hybrid_mode_ == "guided") {
    const auto guided = guidedSearch(start, goal, candidates, cancel_checker);
    if (guided.poses.empty()) {
      throw nav2_core::NoValidPathCouldBeFound(
              "Guided search found no route from start to goal");
    }
    return guided;
  }

  // Dispose + select: keep the shortest collision-free candidate.
  const nav2_diffusion_core::PathCandidate * best = nullptr;
  double best_length = std::numeric_limits<double>::max();
  for (const auto & candidate : candidates) {
    if (cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    if (candidate.size() < 2 || !isPathValid(candidate)) {
      continue;
    }
    const double length = nav2_diffusion_core::pathLength(candidate);
    if (length < best_length) {
      best_length = length;
      best = &candidate;
    }
  }

  if (best == nullptr) {
    // Hybrid: no generative proposal threads the map -> hand off to the classical
    // search, which is complete (e.g. routes through an off-centre gap).
    if (fallback_planner_) {
      RCLCPP_DEBUG(
        logger_, "No valid generative candidate; delegating to fallback '%s'",
        fallback_planner_plugin_.c_str());
      return fallback_planner_->createPlan(start, goal, cancel_checker);
    }
    throw nav2_core::NoValidPathCouldBeFound(
            "No collision-free candidate path among the generated proposals");
  }

  // Build the nav_msgs::Path, orienting each pose toward the next waypoint.
  plan.poses.reserve(best->points.size());
  for (std::size_t i = 0; i < best->points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = best->points[i].x;
    pose.pose.position.y = best->points[i].y;
    double yaw;
    if (i + 1 < best->points.size()) {
      yaw = std::atan2(
        best->points[i + 1].y - best->points[i].y,
        best->points[i + 1].x - best->points[i].x);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      pose.pose.orientation = goal.pose.orientation;  // final pose keeps goal heading
    }
    plan.poses.push_back(pose);
  }

  return plan;
}

nav_msgs::msg::Path DiffusionGlobalPlanner::guidedSearch(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<nav2_diffusion_core::PathCandidate> & proposals,
  std::function<bool()> cancel_checker) const
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = global_frame_;
  auto node = node_.lock();
  plan.header.stamp = node ? node->now() : rclcpp::Clock().now();

  const int sx = static_cast<int>(costmap_->getSizeInCellsX());
  const int sy = static_cast<int>(costmap_->getSizeInCellsY());
  const double res = costmap_->getResolution();
  const std::size_t ncells = static_cast<std::size_t>(sx) * sy;

  unsigned int smx = 0, smy = 0, gmx = 0, gmy = 0;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, smx, smy) ||
    !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gmx, gmy))
  {
    return plan;
  }
  const int gx = static_cast<int>(gmx);
  const int gy = static_cast<int>(gmy);
  const int start_idx = static_cast<int>(smy) * sx + static_cast<int>(smx);
  const int goal_idx = gy * sx + gx;

  // Guidance: cells near a VALID proposal get a cost discount, biasing the search
  // toward the learned corridor where it is free. No valid proposal -> plain A*.
  std::vector<char> near(ncells, 0);
  const int rad = std::max(0, static_cast<int>(std::lround(guidance_radius_ / res)));
  for (const auto & cand : proposals) {
    if (cand.size() < 2 || !isPathValid(cand)) {
      continue;
    }
    for (std::size_t i = 1; i < cand.points.size(); ++i) {
      const auto & a = cand.points[i - 1];
      const auto & b = cand.points[i];
      const double seg = std::hypot(b.x - a.x, b.y - a.y);
      const int samples = std::max(1, static_cast<int>(std::ceil(seg / res)));
      for (int s = 0; s <= samples; ++s) {
        const double t = static_cast<double>(s) / static_cast<double>(samples);
        unsigned int cmx = 0, cmy = 0;
        if (!costmap_->worldToMap(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), cmx, cmy)) {
          continue;
        }
        for (int dy = -rad; dy <= rad; ++dy) {
          for (int dx = -rad; dx <= rad; ++dx) {
            const int nx = static_cast<int>(cmx) + dx;
            const int ny = static_cast<int>(cmy) + dy;
            if (nx >= 0 && ny >= 0 && nx < sx && ny < sy) {
              near[static_cast<std::size_t>(ny) * sx + nx] = 1;
            }
          }
        }
      }
    }
  }

  auto traversable = [&](int mx, int my) {
    const unsigned char c = costmap_->getCost(mx, my);
    if (c == nav2_costmap_2d::LETHAL_OBSTACLE ||
      c == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return false;
    }
    return c != nav2_costmap_2d::NO_INFORMATION || allow_unknown_;
  };
  auto heuristic = [&](int mx, int my) {
    const int dx = std::abs(mx - gx);
    const int dy = std::abs(my - gy);
    const int lo = std::min(dx, dy);
    const int hi = std::max(dx, dy);
    return (hi + (M_SQRT2 - 1.0) * lo) * res;
  };

  std::vector<double> g(ncells, std::numeric_limits<double>::max());
  std::vector<int> came_from(ncells, -1);
  std::vector<char> closed(ncells, 0);
  using Node = std::pair<double, int>;            // (f, idx)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
  g[start_idx] = 0.0;
  open.push({heuristic(static_cast<int>(smx), static_cast<int>(smy)), start_idx});

  const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  int iterations = 0;
  bool found = false;
  while (!open.empty()) {
    if ((++iterations & 0x3ff) == 0 && cancel_checker && cancel_checker()) {
      throw nav2_core::PlannerCancelled("Planning cancelled");
    }
    const int cur = open.top().second;
    open.pop();
    if (closed[cur]) {
      continue;
    }
    closed[cur] = 1;
    if (cur == goal_idx) {
      found = true;
      break;
    }
    const int cx = cur % sx;
    const int cy = cur / sx;
    for (int k = 0; k < 8; ++k) {
      const int nx = cx + dxs[k];
      const int ny = cy + dys[k];
      if (nx < 0 || ny < 0 || nx >= sx || ny >= sy || !traversable(nx, ny)) {
        continue;
      }
      const bool diagonal = dxs[k] != 0 && dys[k] != 0;
      // No corner cutting: a diagonal needs both orthogonal cells free.
      if (diagonal && (!traversable(cx + dxs[k], cy) || !traversable(cx, cy + dys[k]))) {
        continue;
      }
      const int nidx = ny * sx + nx;
      const double base = (diagonal ? M_SQRT2 : 1.0) * res;
      const double factor = near[nidx] ? (1.0 - guidance_strength_) : 1.0;
      const double tentative = g[cur] + base * factor;
      if (tentative < g[nidx]) {
        g[nidx] = tentative;
        came_from[nidx] = cur;
        open.push({tentative + heuristic(nx, ny), nidx});
      }
    }
  }

  if (!found) {
    return plan;          // empty -> no route
  }

  // Backtrack into world-frame poses (cell centres), then snap the endpoints.
  std::vector<int> idx_path;
  for (int n = goal_idx; n != -1; n = came_from[n]) {
    idx_path.push_back(n);
  }
  std::reverse(idx_path.begin(), idx_path.end());

  plan.poses.reserve(idx_path.size());
  std::vector<std::pair<double, double>> pts;
  pts.reserve(idx_path.size());
  for (int n : idx_path) {
    double wx = 0.0, wy = 0.0;
    costmap_->mapToWorld(
      static_cast<unsigned int>(n % sx), static_cast<unsigned int>(n / sx), wx, wy);
    pts.push_back({wx, wy});
  }
  pts.front() = {start.pose.position.x, start.pose.position.y};
  pts.back() = {goal.pose.position.x, goal.pose.position.y};

  for (std::size_t i = 0; i < pts.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = pts[i].first;
    pose.pose.position.y = pts[i].second;
    if (i + 1 < pts.size()) {
      const double yaw = std::atan2(
        pts[i + 1].second - pts[i].second, pts[i + 1].first - pts[i].first);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else {
      pose.pose.orientation = goal.pose.orientation;
    }
    plan.poses.push_back(pose);
  }
  return plan;
}

}  // namespace nav2_diffusion_global_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  nav2_diffusion_global_planner::DiffusionGlobalPlanner, nav2_core::GlobalPlanner)
