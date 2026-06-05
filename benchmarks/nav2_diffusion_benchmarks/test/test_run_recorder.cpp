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

#include "nav2_diffusion_benchmarks/run_recorder.hpp"

using nav2_diffusion_benchmarks::RunRecorder;

TEST(RunRecorderTest, AccumulatesSamples)
{
  RunRecorder recorder;
  EXPECT_TRUE(recorder.empty());
  recorder.addSample(0.0, 0.0, 0.0, 0.0);
  recorder.addSample(1.0, 1.0, 0.0, 0.0);
  EXPECT_EQ(recorder.size(), 2u);
  EXPECT_FALSE(recorder.empty());
}

TEST(RunRecorderTest, ResetClearsSamples)
{
  RunRecorder recorder;
  recorder.addSample(0.0, 0.0, 0.0, 0.0);
  recorder.reset();
  EXPECT_TRUE(recorder.empty());
}

TEST(RunRecorderTest, FinishComputesMetricsFromRecordedPath)
{
  RunRecorder recorder;
  recorder.addSample(0.0, 0.0, 0.0, 0.0);
  recorder.addSample(5.0, 5.0, 0.0, 0.0);

  const auto result = recorder.finish("simple_corridor", "DiffusionController", 5.0, 0.0, 0.25);

  EXPECT_EQ(result.scenario, "simple_corridor");
  EXPECT_EQ(result.controller, "DiffusionController");
  EXPECT_TRUE(result.metrics.reached_goal);
  EXPECT_DOUBLE_EQ(result.metrics.path_length, 5.0);
  EXPECT_DOUBLE_EQ(result.metrics.time_to_goal, 5.0);
}
