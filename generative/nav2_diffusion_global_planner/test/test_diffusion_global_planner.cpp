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

// Integration test for the DiffusionGlobalPlanner: drives the real plugin
// through configure/activate against a live nav2_costmap_2d::Costmap2DROS (no
// Gazebo / GPU) and checks the propose -> validate -> select pipeline: it
// returns a straight path on a clear map, detours around a partial obstacle,
// and fails (throws) when start/goal/corridor are blocked.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_diffusion_global_planner/diffusion_global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

class DiffusionGlobalPlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("diffusion_global_planner_test");

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
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap_ros_->on_configure(rclcpp_lifecycle::State());

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    planner_ = std::make_shared<nav2_diffusion_global_planner::DiffusionGlobalPlanner>();
    planner_->configure(node_, "GridBased", tf_, costmap_ros_);
    planner_->activate();
  }

  static void TearDownTestSuite()
  {
    planner_->deactivate();
    planner_->cleanup();
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    planner_.reset();
    tf_.reset();
    costmap_ros_.reset();
    node_.reset();
    // rclcpp is reclaimed on process exit; re-init in a later suite would hang.
  }

  static void clearCostmap()
  {
    auto * costmap = costmap_ros_->getCostmap();
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
        costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

  // A lethal block of half-width half_cells around the world point (wx, wy).
  static void markLethalBlock(double wx, double wy, int half_cells)
  {
    auto * costmap = costmap_ros_->getCostmap();
    unsigned int cx = 0;
    unsigned int cy = 0;
    ASSERT_TRUE(costmap->worldToMap(wx, wy, cx, cy));
    for (int dy = -half_cells; dy <= half_cells; ++dy) {
      for (int dx = -half_cells; dx <= half_cells; ++dx) {
        costmap->setCost(cx + dx, cy + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  // A full-height lethal wall at world x = wx, spanning the whole costmap.
  static void markLethalWall(double wx, int half_cells)
  {
    auto * costmap = costmap_ros_->getCostmap();
    unsigned int cx = 0;
    unsigned int cy = 0;
    ASSERT_TRUE(costmap->worldToMap(wx, 3.0, cx, cy));
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      for (int dx = -half_cells; dx <= half_cells; ++dx) {
        costmap->setCost(cx + dx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  // A lethal wall at world x = wx spanning the whole height EXCEPT a free gap of
  // half-width gap_half around gap_center.
  static void markWallWithGap(double wx, double gap_center, double gap_half, int half_cells)
  {
    auto * costmap = costmap_ros_->getCostmap();
    unsigned int cx = 0;
    unsigned int cy = 0;
    ASSERT_TRUE(costmap->worldToMap(wx, 3.0, cx, cy));
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      const double wy = costmap->getOriginY() + (my + 0.5) * costmap->getResolution();
      if (std::abs(wy - gap_center) <= gap_half) {
        continue;
      }
      for (int dx = -half_cells; dx <= half_cells; ++dx) {
        costmap->setCost(cx + dx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  static geometry_msgs::msg::PoseStamped pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.w = 1.0;
    return p;
  }

  // Every pose of the plan must be in a traversable (non-lethal) cell.
  static bool planIsCollisionFree(const nav_msgs::msg::Path & plan)
  {
    auto * costmap = costmap_ros_->getCostmap();
    for (const auto & p : plan.poses) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
        return false;
      }
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return false;
      }
    }
    return true;
  }

  static std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static std::shared_ptr<nav2_diffusion_global_planner::DiffusionGlobalPlanner> planner_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> DiffusionGlobalPlannerTest::node_;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> DiffusionGlobalPlannerTest::costmap_ros_;
std::shared_ptr<tf2_ros::Buffer> DiffusionGlobalPlannerTest::tf_;
std::shared_ptr<nav2_diffusion_global_planner::DiffusionGlobalPlanner>
DiffusionGlobalPlannerTest::planner_;

namespace
{
auto noCancel = []() {return false;};
}  // namespace

TEST_F(DiffusionGlobalPlannerTest, ClearMapReturnsStraightPathToGoal)
{
  clearCostmap();
  const auto plan = planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  ASSERT_GE(plan.poses.size(), 2u);
  EXPECT_NEAR(plan.poses.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.x, 5.0, 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.y, 3.0, 1e-6);
  EXPECT_TRUE(planIsCollisionFree(plan));
  // On a clear map the straight (shortest) candidate wins: no lateral deviation.
  for (const auto & p : plan.poses) {
    EXPECT_NEAR(p.pose.position.y, 3.0, 1e-6);
  }
}

TEST_F(DiffusionGlobalPlannerTest, DetoursAroundPartialObstacle)
{
  clearCostmap();
  markLethalBlock(3.0, 3.0, 6);  // ~0.6 m block straddling the straight line
  const auto plan = planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  ASSERT_GE(plan.poses.size(), 2u);
  EXPECT_TRUE(planIsCollisionFree(plan));
  // The chosen path must bow away from the straight line at some point.
  double max_dev = 0.0;
  for (const auto & p : plan.poses) {
    max_dev = std::max(max_dev, std::abs(p.pose.position.y - 3.0));
  }
  EXPECT_GT(max_dev, 0.1);
}

TEST_F(DiffusionGlobalPlannerTest, GoalInLethalCellThrowsGoalOccupied)
{
  clearCostmap();
  markLethalBlock(5.0, 3.0, 4);
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel),
    nav2_core::GoalOccupied);
}

TEST_F(DiffusionGlobalPlannerTest, FullWallThrowsNoValidPath)
{
  clearCostmap();
  markLethalWall(3.0, 3);  // vertical wall spanning the whole height
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel),
    nav2_core::NoValidPathCouldBeFound);
}

TEST_F(DiffusionGlobalPlannerTest, CancelDuringPlanningThrows)
{
  clearCostmap();
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), []() {return true;}),
    nav2_core::PlannerCancelled);
}

// Hybrid: a single-bow analytic/learned proposer cannot make an S-shaped slalom,
// so without a fallback the default planner throws (asserted first). With a
// classical (JPS) fallback configured, the same map yields a complete,
// collision-free path — the "learned proposes, classical search disposes" hybrid.
TEST_F(DiffusionGlobalPlannerTest, HybridFallsBackToClassicalSearch)
{
  clearCostmap();
  markWallWithGap(2.2, 1.0, 0.5, 2);   // gap low
  markWallWithGap(3.8, 5.0, 0.5, 2);   // gap high -> forces an S-shaped detour

  // The default (no-fallback) planner cannot propose the S and gives up.
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel),
    nav2_core::NoValidPathCouldBeFound);

  // A planner with a classical fallback solves the same map.
  if (!node_->has_parameter("HybridPlanner.fallback_planner_plugin")) {
    node_->declare_parameter(
      "HybridPlanner.fallback_planner_plugin",
      rclcpp::ParameterValue(std::string("nav2_jps_planner::JPSPlanner")));
  }
  auto hybrid = std::make_shared<nav2_diffusion_global_planner::DiffusionGlobalPlanner>();
  hybrid->configure(node_, "HybridPlanner", tf_, costmap_ros_);
  hybrid->activate();

  const auto plan = hybrid->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  EXPECT_GE(plan.poses.size(), 2u);
  EXPECT_TRUE(planIsCollisionFree(plan));

  hybrid->deactivate();
  hybrid->cleanup();
}

// Tightly-coupled hybrid (hybrid_mode: guided): a built-in complete A* (cost
// discounted near the proposals) solves the same S-shaped slalom that the pure
// generative proposer cannot, with no external fallback plugin.
TEST_F(DiffusionGlobalPlannerTest, GuidedHybridSolvesSlalomWithCompleteSearch)
{
  clearCostmap();
  markWallWithGap(2.2, 1.0, 0.5, 2);
  markWallWithGap(3.8, 5.0, 0.5, 2);

  if (!node_->has_parameter("GuidedPlanner.hybrid_mode")) {
    node_->declare_parameter(
      "GuidedPlanner.hybrid_mode", rclcpp::ParameterValue(std::string("guided")));
  }
  auto guided = std::make_shared<nav2_diffusion_global_planner::DiffusionGlobalPlanner>();
  guided->configure(node_, "GuidedPlanner", tf_, costmap_ros_);
  guided->activate();

  const auto plan = guided->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  EXPECT_GE(plan.poses.size(), 2u);
  EXPECT_TRUE(planIsCollisionFree(plan));
  EXPECT_NEAR(plan.poses.front().pose.position.x, 1.0, 0.1);
  EXPECT_NEAR(plan.poses.back().pose.position.x, 5.0, 0.1);

  guided->deactivate();
  guided->cleanup();
}
