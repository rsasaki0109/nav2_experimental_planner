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

#include <gtest/gtest.h>

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_diffusion_benchmarks/collision_metrics.hpp"
#include "nav2_diffusion_core/trajectory.hpp"

using nav2_diffusion_benchmarks::evaluateCollisions;
using nav2_diffusion_core::Trajectory;
using nav2_diffusion_core::TrajectoryPoint;

namespace
{

std::vector<geometry_msgs::msg::Point> squareFootprint(double half_size)
{
  std::vector<geometry_msgs::msg::Point> footprint;
  geometry_msgs::msg::Point corner;
  corner.x = half_size;
  corner.y = half_size;
  footprint.push_back(corner);
  corner.x = -half_size;
  corner.y = half_size;
  footprint.push_back(corner);
  corner.x = -half_size;
  corner.y = -half_size;
  footprint.push_back(corner);
  corner.x = half_size;
  corner.y = -half_size;
  footprint.push_back(corner);
  return footprint;
}

// A path along y = 2.5 from x = 2.0 to x = 3.0.
Trajectory horizontalPath()
{
  Trajectory traj;
  for (int i = 0; i <= 10; ++i) {
    traj.points.push_back(TrajectoryPoint{2.0 + 0.1 * i, 2.5, 0.0, 0.1 * i});
  }
  return traj;
}

}  // namespace

TEST(CollisionMetricsTest, ClearPathHasNoCollisionAndSaturatedClearance)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  const auto m = evaluateCollisions(
    horizontalPath(), &costmap, squareFootprint(0.05), 253.0, 2.0);
  EXPECT_FALSE(m.collided);
  EXPECT_EQ(m.collision_count, 0);
  EXPECT_DOUBLE_EQ(m.min_clearance, 2.0);
}

TEST(CollisionMetricsTest, PathThroughObstacleCountsCollision)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  unsigned int mx = 0;
  unsigned int my = 0;
  ASSERT_TRUE(costmap.worldToMap(2.5, 2.5, mx, my));
  for (int dy = -2; dy <= 2; ++dy) {
    for (int dx = -2; dx <= 2; ++dx) {
      costmap.setCost(mx + dx, my + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  const auto m = evaluateCollisions(
    horizontalPath(), &costmap, squareFootprint(0.05), 253.0, 2.0);
  EXPECT_TRUE(m.collided);
  EXPECT_GT(m.collision_count, 0);
  EXPECT_LT(m.min_clearance, 0.1);
}

TEST(CollisionMetricsTest, ClearanceMeasuresDistanceToOffsetObstacle)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  unsigned int mx = 0;
  unsigned int my = 0;
  // A lethal cell 0.3 m above the path (at x = 2.5, y = 2.8).
  ASSERT_TRUE(costmap.worldToMap(2.5, 2.8, mx, my));
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  const auto m = evaluateCollisions(
    horizontalPath(), &costmap, squareFootprint(0.05), 253.0, 2.0);
  EXPECT_FALSE(m.collided);
  EXPECT_NEAR(m.min_clearance, 0.3, 0.05);
}

TEST(CollisionMetricsTest, NullCostmapReturnsDefaults)
{
  const auto m = evaluateCollisions(
    horizontalPath(), nullptr, squareFootprint(0.05), 253.0, 2.0);
  EXPECT_FALSE(m.collided);
  EXPECT_EQ(m.collision_count, 0);
  EXPECT_DOUBLE_EQ(m.min_clearance, 2.0);
}
