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

#include <string>
#include <vector>

#include "nav2_diffusion_core/trajectory_model.hpp"
#include "nav2_diffusion_onnx/onnx_trajectory_model.hpp"

#ifndef ONNX_TEST_MODEL
#define ONNX_TEST_MODEL ""
#endif
#ifndef ONNX_COSTMAP_MODEL
#define ONNX_COSTMAP_MODEL ""
#endif
// The curated, committed Mode A artifact shipped in model_zoo/ (not a build-time
// fixture). Exercising it guards the shipped binary against corruption / drift.
#ifndef ONNX_ZOO_COSTMAP_MODEL
#define ONNX_ZOO_COSTMAP_MODEL ""
#endif
// The transformer Mode A sibling shipped in model_zoo/ (same ONNX contract).
#ifndef ONNX_ZOO_COSTMAP_TRANSFORMER_MODEL
#define ONNX_ZOO_COSTMAP_TRANSFORMER_MODEL ""
#endif
// The recurrent (GRU rollout) Mode A sibling shipped in model_zoo/ (same contract).
#ifndef ONNX_ZOO_COSTMAP_RECURRENT_MODEL
#define ONNX_ZOO_COSTMAP_RECURRENT_MODEL ""
#endif

using nav2_diffusion_onnx::OnnxTrajectoryModel;

namespace
{
// An egocentric 32x32 patch with a one-sided obstacle band. cropEgocentricPatch
// maps col 0 -> +y (left), so obstacle_side > 0 fills the low (left/+y) columns.
nav2_diffusion_core::ModelContext costmapTrajContext(double obstacle_side)
{
  constexpr int kSize = 32;
  nav2_diffusion_core::ModelContext ctx;
  ctx.goal_x = 1.0;
  ctx.linear_speed = 0.3;
  ctx.max_angular_speed = 1.0;
  ctx.time_step = 0.1;
  ctx.costmap_size = kSize;
  ctx.costmap.assign(static_cast<std::size_t>(kSize) * kSize, 0.0f);
  const int c0 = obstacle_side > 0 ? 0 : kSize / 2;
  const int c1 = obstacle_side > 0 ? kSize / 2 : kSize;
  for (int r = kSize / 4; r < kSize / 2; ++r) {
    for (int c = c0; c < c1; ++c) {
      ctx.costmap[static_cast<std::size_t>(r) * kSize + c] = 1.0f;
    }
  }
  return ctx;
}

double meanLateral(const nav2_diffusion_core::Trajectory & t)
{
  double sum = 0.0;
  for (const auto & p : t.points) {
    sum += p.y;
  }
  return t.points.empty() ? 0.0 : sum / static_cast<double>(t.points.size());
}

// Load a shipped model_zoo Mode A artifact and assert its headline behaviour:
// every proposed trajectory veers away from a one-sided obstacle.
void expectVeersAwayFromObstacle(const std::string & path)
{
  OnnxTrajectoryModel model(path);
  EXPECT_EQ(model.name(), "onnx");
  // Obstacle on the +y (left) side -> every candidate should lean -y (right).
  const auto left = model.generate(costmapTrajContext(+1.0));
  ASSERT_FALSE(left.empty());
  for (const auto & t : left) {
    EXPECT_LT(meanLateral(t), 0.0);
  }
  // Obstacle on the -y (right) side -> every candidate should lean +y (left).
  const auto right = model.generate(costmapTrajContext(-1.0));
  ASSERT_FALSE(right.empty());
  for (const auto & t : right) {
    EXPECT_GT(meanLateral(t), 0.0);
  }
}
}  // namespace

TEST(OnnxTrajectoryModelTest, LoadsModelAndProducesCandidates)
{
  OnnxTrajectoryModel model;
  model.configure(ONNX_TEST_MODEL);
  EXPECT_EQ(model.name(), "onnx");

  nav2_diffusion_core::ModelContext context;
  context.goal_x = 2.0;
  context.goal_y = 0.5;
  context.linear_speed = 0.3;
  context.max_angular_speed = 1.0;
  context.time_step = 0.1;

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
  OnnxTrajectoryModel model(ONNX_TEST_MODEL);
  nav2_diffusion_core::ModelContext context;
  context.goal_x = 1.0;
  context.time_step = 0.1;

  const auto a = model.generate(context);
  const auto b = model.generate(context);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_FALSE(a.empty());
  EXPECT_DOUBLE_EQ(a[0].points[0].x, b[0].points[0].x);
  EXPECT_DOUBLE_EQ(a[0].points.back().y, b[0].points.back().y);
}

TEST(OnnxTrajectoryModelTest, CostmapConditionedModelRuns)
{
  // A model exporting a "costmap" input is fed the egocentric patch from the
  // ModelContext and still produces the K x H x 3 candidate set.
  OnnxTrajectoryModel model(ONNX_COSTMAP_MODEL);
  nav2_diffusion_core::ModelContext context;
  context.goal_x = 1.0;
  context.time_step = 0.1;
  context.costmap_size = 16;
  context.costmap.assign(16 * 16, 0.0f);
  for (int r = 4; r < 8; ++r) {
    for (int c = 0; c < 8; ++c) {
      context.costmap[r * 16 + c] = 1.0f;  // obstacle on one side
    }
  }

  const auto candidates = model.generate(context);
  ASSERT_EQ(candidates.size(), 3u);
  EXPECT_EQ(candidates.front().points.size(), 10u);
}

// The shipped model_zoo artifact (diffusion_local_costmap_flow_v0): load the
// actual committed .onnx and assert its headline behaviour — every proposed
// trajectory veers away from a one-sided obstacle. This is the repo's first
// learned local-controller model in the loop, so the curated binary is a
// regression guard.
TEST(OnnxTrajectoryModelTest, CuratedZooModelVeersAwayFromObstacle)
{
  const std::string zoo = ONNX_ZOO_COSTMAP_MODEL;
  if (zoo.empty()) {
    GTEST_SKIP() << "model_zoo costmap trajectory model path not provided";
  }
  expectVeersAwayFromObstacle(zoo);
}

// The transformer Mode A sibling (diffusion_local_costmap_transformer_v0): same
// ONNX contract, same headline behaviour. Guards the second shipped binary.
TEST(OnnxTrajectoryModelTest, CuratedZooTransformerVeersAwayFromObstacle)
{
  const std::string zoo = ONNX_ZOO_COSTMAP_TRANSFORMER_MODEL;
  if (zoo.empty()) {
    GTEST_SKIP() << "model_zoo transformer trajectory model path not provided";
  }
  expectVeersAwayFromObstacle(zoo);
}

// The recurrent Mode A sibling (diffusion_local_costmap_recurrent_v0): same ONNX
// contract, same headline behaviour. Guards the third shipped binary.
TEST(OnnxTrajectoryModelTest, CuratedZooRecurrentVeersAwayFromObstacle)
{
  const std::string zoo = ONNX_ZOO_COSTMAP_RECURRENT_MODEL;
  if (zoo.empty()) {
    GTEST_SKIP() << "model_zoo recurrent trajectory model path not provided";
  }
  expectVeersAwayFromObstacle(zoo);
}
