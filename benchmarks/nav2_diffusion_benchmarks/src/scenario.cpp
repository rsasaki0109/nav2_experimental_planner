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

#include "nav2_diffusion_benchmarks/scenario.hpp"

#include <string>

#include "yaml-cpp/yaml.h"

namespace nav2_diffusion_benchmarks
{

namespace
{
ScenarioPose parsePose(const YAML::Node & node)
{
  ScenarioPose pose;
  if (!node) {
    return pose;
  }
  pose.x = node["x"].as<double>(0.0);
  pose.y = node["y"].as<double>(0.0);
  pose.yaw = node["yaw"].as<double>(0.0);
  return pose;
}
}  // namespace

Scenario parseScenario(const std::string & yaml_text)
{
  const YAML::Node node = YAML::Load(yaml_text);
  Scenario scenario;
  scenario.name = node["name"].as<std::string>("");
  scenario.map = node["map"].as<std::string>("");
  scenario.robot = node["robot"].as<std::string>("");
  scenario.start = parsePose(node["start"]);
  scenario.goal = parsePose(node["goal"]);
  scenario.goal_tolerance = node["goal_tolerance"].as<double>(0.25);
  scenario.seed = node["seed"].as<int>(0);
  return scenario;
}

Scenario loadScenarioFile(const std::string & path)
{
  const YAML::Node node = YAML::LoadFile(path);
  YAML::Emitter emitter;
  emitter << node;
  return parseScenario(emitter.c_str());
}

}  // namespace nav2_diffusion_benchmarks
