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

#include <vector>

#include "nav2_diffusion_core/fan_rollout_model.hpp"
#include "nav2_diffusion_core/trajectory_model.hpp"

using nav2_diffusion_core::FanRolloutModel;
using nav2_diffusion_core::ModelContext;

namespace
{

ModelContext straightAheadContext(int num_candidates)
{
  ModelContext ctx;
  ctx.goal_x = 1.0;
  ctx.goal_y = 0.0;
  ctx.linear_speed = 0.3;
  ctx.max_angular_speed = 1.0;
  ctx.horizon = 2.0;
  ctx.time_step = 0.1;
  ctx.num_candidates = num_candidates;
  return ctx;
}

}  // namespace

TEST(FanRolloutModelTest, GeneratesRequestedNumberOfNonEmptyCandidates)
{
  FanRolloutModel model;
  const auto candidates = model.generate(straightAheadContext(5));
  ASSERT_EQ(candidates.size(), 5u);
  for (const auto & candidate : candidates) {
    EXPECT_GE(candidate.points.size(), 2u);
  }
}

TEST(FanRolloutModelTest, FanSpreadsLeftToRight)
{
  FanRolloutModel model;
  const auto candidates = model.generate(straightAheadContext(5));
  // The first candidate turns hard right (endpoint y < 0), the last hard left.
  EXPECT_LT(candidates.front().points.back().y, candidates.back().points.back().y);
}

TEST(FanRolloutModelTest, SingleCandidateUsesNominalHeading)
{
  FanRolloutModel model;
  // Goal straight ahead -> nominal angular ~0 -> near-straight trajectory.
  const auto candidates = model.generate(straightAheadContext(1));
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_NEAR(candidates.front().points.back().y, 0.0, 1e-9);
  EXPECT_GT(candidates.front().points.back().x, 0.0);
}
