// Copyright 2026 nav2_diffusion_planner contributors
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

#include "nav2_diffusion_core/trajectory_model.hpp"
#include "nav2_diffusion_onnx/onnx_trajectory_model.hpp"

#ifndef ONNX_TEST_MODEL
#define ONNX_TEST_MODEL ""
#endif

using nav2_diffusion_onnx::OnnxTrajectoryModel;

TEST(OnnxTrajectoryModelTest, LoadsModelAndProducesCandidates)
{
  OnnxTrajectoryModel model(ONNX_TEST_MODEL, 0.1);
  EXPECT_EQ(model.name(), "onnx");

  nav2_diffusion_core::ModelContext context;
  context.goal_x = 2.0;
  context.goal_y = 0.5;
  context.linear_speed = 0.3;
  context.max_angular_speed = 1.0;

  const auto candidates = model.generate(context);

  // The fixture model emits K=3 candidates of H=10 steps each.
  ASSERT_EQ(candidates.size(), 3u);
  for (const auto & candidate : candidates) {
    ASSERT_EQ(candidate.points.size(), 10u);
    EXPECT_DOUBLE_EQ(candidate.points.front().time, 0.0);
    EXPECT_DOUBLE_EQ(candidate.points[1].time, 0.1);
  }
}

TEST(OnnxTrajectoryModelTest, IsDeterministic)
{
  OnnxTrajectoryModel model(ONNX_TEST_MODEL, 0.1);
  nav2_diffusion_core::ModelContext context;
  context.goal_x = 1.0;

  const auto a = model.generate(context);
  const auto b = model.generate(context);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_FALSE(a.empty());
  EXPECT_DOUBLE_EQ(a[0].points[0].x, b[0].points[0].x);
  EXPECT_DOUBLE_EQ(a[0].points.back().y, b[0].points.back().y);
}
