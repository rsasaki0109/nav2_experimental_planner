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

// Closed-loop integration test for the VFHController: drives the real plugin
// through configure/activate against a live nav2_costmap_2d::Costmap2DROS (no
// Gazebo / GPU). Checks it drives straight on a clear path, steers around an
// obstacle dead ahead (while still moving), stops without a plan, and stops at
// the goal.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_vfh_controller/vfh_controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{
constexpr double kRobotX = 2.5;
constexpr double kRobotY = 2.5;
}  // namespace

class VFHControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("vfh_controller_test");

    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("width", 5),
      rclcpp::Parameter("height", 5),
      rclcpp::Parameter("resolution", 0.05),
      rclcpp::Parameter("robot_radius", 0.15),
      rclcpp::Parameter("global_frame", std::string("map")),
      rclcpp::Parameter("robot_base_frame", std::string("base_link")),
      rclcpp::Parameter("rolling_window", false),
      rclcpp::Parameter("track_unknown_space", false),
    });
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap_ros_->on_configure(rclcpp_lifecycle::State());

    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setUsingDedicatedThread(true);
    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.frame_id = "map";
    map_to_base.child_frame_id = "base_link";
    map_to_base.transform.translation.x = kRobotX;
    map_to_base.transform.translation.y = kRobotY;
    map_to_base.transform.rotation.w = 1.0;
    tf_->setTransform(map_to_base, "test", true);

    controller_ = std::make_shared<nav2_vfh_controller::VFHController>();
    controller_->configure(node_, "FollowPath", tf_, costmap_ros_);
    controller_->activate();
  }

  static void TearDownTestSuite()
  {
    controller_->deactivate();
    controller_->cleanup();
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    controller_.reset();
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

  static void markLethalAt(double wx, double wy, int half_cells)
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

  static geometry_msgs::msg::PoseStamped robotPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = kRobotX;
    pose.pose.position.y = kRobotY;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  static nav_msgs::msg::Path straightPlan()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 1; i <= 20; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = kRobotX + 0.1 * i;
      pose.pose.position.y = kRobotY;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  static std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static std::shared_ptr<nav2_vfh_controller::VFHController> controller_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> VFHControllerTest::node_;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS> VFHControllerTest::costmap_ros_;
std::shared_ptr<tf2_ros::Buffer> VFHControllerTest::tf_;
std::shared_ptr<nav2_vfh_controller::VFHController> VFHControllerTest::controller_;

TEST_F(VFHControllerTest, DrivesForwardOnClearPath)
{
  clearCostmap();
  controller_->setPlan(straightPlan());
  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_NEAR(cmd.twist.angular.z, 0.0, 1e-3);
}

TEST_F(VFHControllerTest, SteersAroundObstacleAhead)
{
  clearCostmap();
  // Lethal block dead ahead (sides left clear): VFH should turn to a free valley
  // while still advancing.
  markLethalAt(kRobotX + 0.6, kRobotY, 3);
  controller_->setPlan(straightPlan());
  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_GT(std::abs(cmd.twist.angular.z), 0.05);  // steered off the blocked heading
  EXPECT_GE(cmd.twist.linear.x, 0.0);              // never reverses
}

TEST_F(VFHControllerTest, StopsWhenNoPlan)
{
  clearCostmap();
  controller_->setPlan(nav_msgs::msg::Path());
  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}

TEST_F(VFHControllerTest, StopsAtGoal)
{
  clearCostmap();
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = kRobotX + 0.05;  // goal essentially under the robot
  p.pose.position.y = kRobotY;
  p.pose.orientation.w = 1.0;
  path.poses.push_back(p);
  controller_->setPlan(path);
  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);
  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}
