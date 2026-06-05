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

// Closed-loop integration test for the DiffusionController: drives the real
// plugin through configure/activate against a live nav2_costmap_2d::Costmap2DROS
// (no Gazebo / GPU required) and checks that it moves forward on a clear path
// and stops when its footprint would hit a lethal obstacle.
//
// The costmap and controller are created once per suite, because constructing
// and destroying a Costmap2DROS lifecycle node is expensive.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_diffusion_controller/diffusion_controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{
constexpr double kRobotX = 2.5;  // robot placed at the costmap centre (map frame)
constexpr double kRobotY = 2.5;
}  // namespace

class DiffusionControllerIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("diffusion_controller_test");

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
    tf_->setUsingDedicatedThread(true);  // we populate it synchronously with static data
    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.frame_id = "map";
    map_to_base.child_frame_id = "base_link";
    map_to_base.transform.translation.x = kRobotX;
    map_to_base.transform.translation.y = kRobotY;
    map_to_base.transform.rotation.w = 1.0;
    tf_->setTransform(map_to_base, "test", true);

    controller_ = std::make_shared<nav2_diffusion_controller::DiffusionController>();
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
    // rclcpp is shared across fixtures and reclaimed on process exit;
    // do not shut it down here (a re-init in a later suite would hang).
  }

  // Reset the whole costmap to free space.
  static void clearCostmap()
  {
    auto * costmap = costmap_ros_->getCostmap();
    for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
      for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
        costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
      }
    }
  }

  // Place a lethal block centred on the given world coordinate.
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

  // A straight global plan heading in +x from the robot.
  static nav_msgs::msg::Path straightPlan()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 1; i <= 15; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = kRobotX + 0.1 * i;
      pose.pose.position.y = kRobotY;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  // A global plan curving up and to the left (+x, +y) from the robot.
  static nav_msgs::msg::Path leftCurvingPlan()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 1; i <= 15; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = kRobotX + 0.07 * i;
      pose.pose.position.y = kRobotY + 0.07 * i;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  static std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static std::shared_ptr<nav2_diffusion_controller::DiffusionController> controller_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
DiffusionControllerIntegrationTest::node_ = nullptr;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
DiffusionControllerIntegrationTest::costmap_ros_ = nullptr;
std::shared_ptr<tf2_ros::Buffer>
DiffusionControllerIntegrationTest::tf_ = nullptr;
std::shared_ptr<nav2_diffusion_controller::DiffusionController>
DiffusionControllerIntegrationTest::controller_ = nullptr;

TEST_F(DiffusionControllerIntegrationTest, DrivesForwardOnClearPath)
{
  clearCostmap();
  controller_->setPlan(straightPlan());

  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_NEAR(cmd.twist.angular.z, 0.0, 1e-6);
}

TEST_F(DiffusionControllerIntegrationTest, StopsWhenObstacleBlocksPath)
{
  clearCostmap();
  // Lethal wall ~0.4 m ahead of the robot, within the rollout horizon.
  markLethalAt(kRobotX + 0.4, kRobotY, 4);
  controller_->setPlan(straightPlan());

  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
}

TEST_F(DiffusionControllerIntegrationTest, StopsWhenNoPlan)
{
  clearCostmap();
  controller_->setPlan(nav_msgs::msg::Path());

  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

TEST_F(DiffusionControllerIntegrationTest, SelectsTurningCandidateForOffAxisGoal)
{
  // With a clear costmap and a goal up-and-to-the-left, the multimodal scorer
  // should pick a left-turning candidate (positive angular) that still advances.
  clearCostmap();
  controller_->setPlan(leftCurvingPlan());

  const auto cmd = controller_->computeVelocityCommands(
    robotPose(), geometry_msgs::msg::Twist(), nullptr);

  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_GT(cmd.twist.angular.z, 0.0);
}

TEST_F(DiffusionControllerIntegrationTest, StopsOnStalePose)
{
  clearCostmap();
  controller_->setPlan(straightPlan());

  // A pose timestamped well beyond the default data_timeout (0.5 s).
  geometry_msgs::msg::PoseStamped stale = robotPose();
  stale.header.stamp = rclcpp::Time(node_->now()) - rclcpp::Duration::from_seconds(5.0);

  const auto cmd = controller_->computeVelocityCommands(
    stale, geometry_msgs::msg::Twist(), nullptr);

  EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
}

// Separate fixture: controller configured with an RPP fallback controller.
class DiffusionControllerFallbackTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    rclcpp::NodeOptions node_opts;
    node_opts.parameter_overrides({
      rclcpp::Parameter(
        "FollowPathFb.fallback_controller_plugin",
        std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")),
      rclcpp::Parameter("FollowPathFb.fallback.use_collision_detection", false),
      rclcpp::Parameter("FollowPathFb.fallback.desired_linear_velocity", 0.26),
    });
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "diffusion_controller_fb_test", node_opts);

    rclcpp::NodeOptions cm_opts;
    cm_opts.parameter_overrides({
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
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(cm_opts);
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

    controller_ = std::make_shared<nav2_diffusion_controller::DiffusionController>();
    controller_->configure(node_, "FollowPathFb", tf_, costmap_ros_);
    controller_->activate();

    // A real goal checker, as controller_server always provides one.
    gc_loader_ = std::make_shared<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
      "nav2_core", "nav2_core::GoalChecker");
    goal_checker_ = gc_loader_->createSharedInstance("nav2_controller::SimpleGoalChecker");
    goal_checker_->initialize(node_, "goal_checker", costmap_ros_);
  }

  static void TearDownTestSuite()
  {
    controller_->deactivate();
    controller_->cleanup();
    costmap_ros_->on_cleanup(rclcpp_lifecycle::State());
    goal_checker_.reset();
    gc_loader_.reset();
    controller_.reset();
    tf_.reset();
    costmap_ros_.reset();
    node_.reset();
    // rclcpp is shared across fixtures and reclaimed on process exit;
    // do not shut it down here (a re-init in a later suite would hang).
  }

  static std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  static std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  static std::shared_ptr<tf2_ros::Buffer> tf_;
  static std::shared_ptr<nav2_diffusion_controller::DiffusionController> controller_;
  static std::shared_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>> gc_loader_;
  static nav2_core::GoalChecker::Ptr goal_checker_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
DiffusionControllerFallbackTest::node_ = nullptr;
std::shared_ptr<nav2_costmap_2d::Costmap2DROS>
DiffusionControllerFallbackTest::costmap_ros_ = nullptr;
std::shared_ptr<tf2_ros::Buffer>
DiffusionControllerFallbackTest::tf_ = nullptr;
std::shared_ptr<nav2_diffusion_controller::DiffusionController>
DiffusionControllerFallbackTest::controller_ = nullptr;
std::shared_ptr<pluginlib::ClassLoader<nav2_core::GoalChecker>>
DiffusionControllerFallbackTest::gc_loader_ = nullptr;
nav2_core::GoalChecker::Ptr
DiffusionControllerFallbackTest::goal_checker_ = nullptr;

TEST_F(DiffusionControllerFallbackTest, DelegatesToFallbackWhenAllCandidatesBlocked)
{
  auto * costmap = costmap_ros_->getCostmap();
  for (unsigned int my = 0; my < costmap->getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < costmap->getSizeInCellsX(); ++mx) {
      costmap->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
    }
  }
  // Wall blocking every generative candidate just ahead of the robot.
  unsigned int cx = 0;
  unsigned int cy = 0;
  ASSERT_TRUE(costmap->worldToMap(kRobotX + 0.4, kRobotY, cx, cy));
  for (int dy = -8; dy <= 8; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      costmap->setCost(cx + dx, cy + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 1; i <= 15; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = kRobotX + 0.1 * i;
    pose.pose.position.y = kRobotY;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  controller_->setPlan(path);

  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = "map";
  robot.pose.position.x = kRobotX;
  robot.pose.position.y = kRobotY;
  robot.pose.orientation.w = 1.0;

  // Our generative gate would stop here; with the fallback configured, control
  // is delegated to RPP, which keeps driving toward the path.
  const auto cmd = controller_->computeVelocityCommands(
    robot, geometry_msgs::msg::Twist(), goal_checker_.get());

  EXPECT_GT(cmd.twist.linear.x, 0.0);
}
