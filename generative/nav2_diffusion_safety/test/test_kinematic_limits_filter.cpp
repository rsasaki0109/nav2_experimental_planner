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

#include <gtest/gtest.h>

#include "nav2_diffusion_core/trajectory.hpp"
#include "nav2_diffusion_safety/kinematic_limits_filter.hpp"

using nav2_diffusion_core::Trajectory;
using nav2_diffusion_core::TrajectoryPoint;
using nav2_diffusion_safety::KinematicLimitsFilter;

TEST(KinematicLimitsFilterTest, AcceptsTrajectoryWithinLimits)
{
  KinematicLimitsFilter filter(1.0, 1.0);
  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{0.5, 0.0, 0.0, 1.0},
  };
  EXPECT_TRUE(filter.check(traj).safe);
}

TEST(KinematicLimitsFilterTest, RejectsTrajectoryExceedingLinearSpeed)
{
  KinematicLimitsFilter filter(1.0, 10.0);
  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{5.0, 0.0, 0.0, 1.0},
  };
  const auto result = filter.check(traj);
  EXPECT_FALSE(result.safe);
  EXPECT_FALSE(result.rejection_reason.empty());
}

TEST(KinematicLimitsFilterTest, RejectsNonIncreasingTime)
{
  KinematicLimitsFilter filter(10.0, 10.0);
  Trajectory traj;
  traj.points = {
    TrajectoryPoint{0.0, 0.0, 0.0, 0.0},
    TrajectoryPoint{0.1, 0.0, 0.0, 0.0},
  };
  EXPECT_FALSE(filter.check(traj).safe);
}
