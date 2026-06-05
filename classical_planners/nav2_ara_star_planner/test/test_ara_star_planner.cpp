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

// Integration test for the ARAStarPlanner: drives the real plugin through
// configure/activate against a live nav2_costmap_2d::Costmap2DROS (no Gazebo /
// GPU). Covers the usual clear-map / off-centre-gap / blocked-wall / occupied-
// goal / cancel cases, plus an inflated-epsilon configuration to exercise the
// anytime weighted-A* / improvement loop.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_ara_star_planner/ara_star_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

class ARAStarPlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ara_star_planner_test");

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

    planner_ = std::make_shared<nav2_ara_star_planner::ARAStarPlanner>();
    planner_->configure(node_, "GridBased", tf_, costmap_ros_);
    planner_->activate();

    // A second instance with a large starting inflation factor to exercise the
    // anytime weighted-A* / epsilon-improvement loop.
    node_->declare_parameter("Inflated.initial_epsilon", 5.0);
    planner_inflated_ = std::make_shared<nav2_ara_star_planner::ARAStarPlanner>();
    planner_inflated_->configure(node_, "Inflated", tf_, costmap_ros_);
    planner_inflated_->activate();
  }

  static void TearDownTestSuite()
  {
    planner_->deactivate();
    planner_->cleanup();
    planner_inflated_->deactivate();
    planner_inflated_->cleanup();
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    planner_.reset();
    planner_inflated_.reset();
    tf_.reset();
    costmap_ros_.reset();
    node_.reset();
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

  // Vertical lethal wall at world x = wx (a few cells thick). Cells whose world
  // y is within gap_half of gap_center are left free (the gap). gap_half <= 0
  // gives a solid wall.
  static void markWall(double wx, double gap_center, double gap_half, int half_cells = 2)
  {
    auto * costmap = costmap_ros_->getCostmap();
    unsigned int cx = 0;
    unsigned int cy = 0;
    ASSERT_TRUE(costmap->worldToMap(wx, 3.0, cx, cy));
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      const double wy = costmap->getOriginY() + (my + 0.5) * costmap->getResolution();
      if (gap_half > 0.0 && std::abs(wy - gap_center) <= gap_half) {
        continue;  // leave the gap free
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
  static std::shared_ptr<nav2_ara_star_planner::ARAStarPlanner> planner_;
  static std::shared_ptr<nav2_ara_star_planner::ARAStarPlanner> planner_inflated_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ARAStarPlannerTest::node_;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ARAStarPlannerTest::costmap_ros_;
std::shared_ptr<tf2_ros::Buffer> ARAStarPlannerTest::tf_;
std::shared_ptr<nav2_ara_star_planner::ARAStarPlanner> ARAStarPlannerTest::planner_;
std::shared_ptr<nav2_ara_star_planner::ARAStarPlanner> ARAStarPlannerTest::planner_inflated_;

namespace
{
auto noCancel = []() {return false;};
}  // namespace

TEST_F(ARAStarPlannerTest, ClearMapReturnsCollisionFreePathToGoal)
{
  clearCostmap();
  const auto plan = planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  ASSERT_GE(plan.poses.size(), 2u);
  EXPECT_NEAR(plan.poses.front().pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(plan.poses.front().pose.position.y, 3.0, 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.x, 5.0, 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.y, 3.0, 1e-6);
  EXPECT_TRUE(planIsCollisionFree(plan));
}

TEST_F(ARAStarPlannerTest, InflatedEpsilonReturnsValidPath)
{
  clearCostmap();
  const auto plan = planner_inflated_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  ASSERT_GE(plan.poses.size(), 2u);
  EXPECT_NEAR(plan.poses.back().pose.position.x, 5.0, 1e-6);
  EXPECT_NEAR(plan.poses.back().pose.position.y, 3.0, 1e-6);
  EXPECT_TRUE(planIsCollisionFree(plan));
}

TEST_F(ARAStarPlannerTest, RoutesThroughOffCentreGap)
{
  clearCostmap();
  markWall(3.0, /*gap_center=*/1.0, /*gap_half=*/0.4);  // gap well off the straight line
  const auto plan = planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel);
  ASSERT_GE(plan.poses.size(), 2u);
  EXPECT_TRUE(planIsCollisionFree(plan));
  double min_y = 1e9;
  for (const auto & p : plan.poses) {
    min_y = std::min(min_y, p.pose.position.y);
  }
  EXPECT_LT(min_y, 1.6);
}

TEST_F(ARAStarPlannerTest, SolidWallThrowsNoValidPath)
{
  clearCostmap();
  markWall(3.0, 0.0, /*gap_half=*/0.0);  // no gap
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel),
    nav2_core::NoValidPathCouldBeFound);
}

TEST_F(ARAStarPlannerTest, GoalInLethalCellThrowsGoalOccupied)
{
  clearCostmap();
  markWall(5.0, 0.0, 0.0, /*half_cells=*/3);
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), noCancel),
    nav2_core::GoalOccupied);
}

TEST_F(ARAStarPlannerTest, CancelDuringPlanningThrows)
{
  clearCostmap();
  EXPECT_THROW(
    planner_->createPlan(pose(1.0, 3.0), pose(5.0, 3.0), []() {return true;}),
    nav2_core::PlannerCancelled);
}
