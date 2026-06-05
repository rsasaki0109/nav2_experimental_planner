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

#include "nav2_diffusion_core/trajectory.hpp"

using nav2_diffusion_core::Trajectory;
using nav2_diffusion_core::TrajectoryPoint;

TEST(TrajectoryTest, EmptyTrajectoryHasZeroLengthAndDuration)
{
  Trajectory traj;
  EXPECT_TRUE(traj.empty());
  EXPECT_DOUBLE_EQ(nav2_diffusion_core::pathLength(traj), 0.0);
  EXPECT_DOUBLE_EQ(nav2_diffusion_core::duration(traj), 0.0);
}

TEST(TrajectoryTest, PathLengthSumsSegmentDistances)
{
  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{3.0, 0.0, 0.0, 1.0},
    TrajectoryPoint{3.0, 4.0, 0.0, 2.0},
  };
  EXPECT_DOUBLE_EQ(nav2_diffusion_core::pathLength(traj), 7.0);
  EXPECT_DOUBLE_EQ(nav2_diffusion_core::duration(traj), 2.0);
}
