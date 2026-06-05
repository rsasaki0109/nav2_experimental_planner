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

#ifndef NAV2_DIFFUSION_BENCHMARKS__SCENARIO_HPP_
#define NAV2_DIFFUSION_BENCHMARKS__SCENARIO_HPP_

#include <string>

namespace nav2_diffusion_benchmarks
{

/// SE(2) pose used in a scenario definition.
struct ScenarioPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// A reproducible benchmark scenario (docs/benchmarking.md section 9.3,
/// docs/simulation.md section 10.3). The seed pins dynamic-obstacle initial
/// conditions so runs are comparable across controllers.
struct Scenario
{
  std::string name;
  std::string map;
  std::string robot;
  ScenarioPose start;
  ScenarioPose goal;
  double goal_tolerance{0.25};
  int seed{0};
};

/// Parse a scenario from a YAML string. Missing fields take their defaults.
Scenario parseScenario(const std::string & yaml_text);

/// Load a scenario from a YAML file (thin wrapper over parseScenario).
Scenario loadScenarioFile(const std::string & path);

}  // namespace nav2_diffusion_benchmarks

#endif  // NAV2_DIFFUSION_BENCHMARKS__SCENARIO_HPP_
