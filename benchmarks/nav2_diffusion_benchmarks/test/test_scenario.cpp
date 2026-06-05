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

#include <fstream>
#include <string>

#include "nav2_diffusion_benchmarks/scenario.hpp"

using nav2_diffusion_benchmarks::loadScenarioFile;
using nav2_diffusion_benchmarks::parseScenario;

TEST(ScenarioTest, ParsesAllFields)
{
  const std::string yaml =
    "name: narrow_doorway\n"
    "map: tb3_sandbox\n"
    "robot: turtlebot3_waffle\n"
    "start: {x: 0.0, y: 0.0, yaw: 0.0}\n"
    "goal: {x: 2.5, y: -1.0, yaw: 1.57}\n"
    "goal_tolerance: 0.2\n"
    "seed: 42\n";
  const auto scenario = parseScenario(yaml);
  EXPECT_EQ(scenario.name, "narrow_doorway");
  EXPECT_EQ(scenario.map, "tb3_sandbox");
  EXPECT_EQ(scenario.robot, "turtlebot3_waffle");
  EXPECT_DOUBLE_EQ(scenario.goal.x, 2.5);
  EXPECT_DOUBLE_EQ(scenario.goal.y, -1.0);
  EXPECT_DOUBLE_EQ(scenario.goal.yaw, 1.57);
  EXPECT_DOUBLE_EQ(scenario.goal_tolerance, 0.2);
  EXPECT_EQ(scenario.seed, 42);
}

TEST(ScenarioTest, MissingFieldsTakeDefaults)
{
  const auto scenario = parseScenario("name: minimal\n");
  EXPECT_EQ(scenario.name, "minimal");
  EXPECT_DOUBLE_EQ(scenario.goal.x, 0.0);
  EXPECT_DOUBLE_EQ(scenario.goal_tolerance, 0.25);
  EXPECT_EQ(scenario.seed, 0);
}

TEST(ScenarioTest, LoadsFromFile)
{
  const std::string path = std::string(::testing::TempDir()) + "/scenario_test.yaml";
  {
    std::ofstream file(path);
    file << "name: from_file\ngoal: {x: 1.0, y: 2.0}\nseed: 7\n";
  }
  const auto scenario = loadScenarioFile(path);
  EXPECT_EQ(scenario.name, "from_file");
  EXPECT_DOUBLE_EQ(scenario.goal.x, 1.0);
  EXPECT_DOUBLE_EQ(scenario.goal.y, 2.0);
  EXPECT_EQ(scenario.seed, 7);
}
