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

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_diffusion_core/trajectory.hpp"
#include "nav2_diffusion_safety/footprint_collision_filter.hpp"

using nav2_diffusion_core::Trajectory;
using nav2_diffusion_core::TrajectoryPoint;
using nav2_diffusion_safety::FootprintCollisionFilter;

namespace
{

FootprintCollisionFilter::Footprint makeSquareFootprint(double half_size)
{
  FootprintCollisionFilter::Footprint footprint;
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

}  // namespace

TEST(FootprintCollisionFilterTest, AcceptsTrajectoryOverFreeSpace)
{
  // 10x10 cells, 0.1 m/cell, free space.
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  FootprintCollisionFilter filter(&costmap, makeSquareFootprint(0.05));

  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.2, 0.2, 0.0, 0.0},
    TrajectoryPoint{0.3, 0.2, 0.0, 1.0},
  };
  EXPECT_TRUE(filter.check(traj).safe);
}

TEST(FootprintCollisionFilterTest, RejectsTrajectoryOverLethalCell)
{
  nav2_costmap_2d::Costmap2D costmap(10, 10, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  FootprintCollisionFilter filter(&costmap, makeSquareFootprint(0.05));

  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.2, 0.2, 0.0, 0.0},
    TrajectoryPoint{0.5, 0.5, 0.0, 1.0},
  };
  const auto result = filter.check(traj);
  EXPECT_FALSE(result.safe);
  EXPECT_FALSE(result.rejection_reason.empty());
}

TEST(FootprintCollisionFilterTest, NullCostmapIsRejected)
{
  FootprintCollisionFilter filter(nullptr, makeSquareFootprint(0.05));
  Trajectory traj;
  traj.points = {TrajectoryPoint{0.1, 0.1, 0.0, 0.0}};
  EXPECT_FALSE(filter.check(traj).safe);
}
