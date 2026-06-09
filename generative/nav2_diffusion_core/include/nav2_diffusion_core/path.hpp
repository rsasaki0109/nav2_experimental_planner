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

#ifndef NAV2_DIFFUSION_CORE__PATH_HPP_
#define NAV2_DIFFUSION_CORE__PATH_HPP_

#include <cstddef>
#include <vector>

namespace nav2_diffusion_core
{

/// A single waypoint of a global path, in the planner (map/world) frame.
struct PathPoint
{
  double x{0.0};  ///< metres
  double y{0.0};  ///< metres
};

/// A candidate global path: an ordered start->goal waypoint sequence.
///
/// This is the global (Mode B) analogue of Trajectory: a generative model
/// proposes K of these, and a deterministic validity layer (collision check
/// against the global costmap) decides which, if any, is executed. The
/// model_score is a soft prior only and never implies a path is safe.
struct PathCandidate
{
  std::vector<PathPoint> points;
  double model_score{0.0};

  bool empty() const {return points.empty();}
  std::size_t size() const {return points.size();}
};

/// Total polyline length of the path, in metres.
double pathLength(const PathCandidate & path);

}  // namespace nav2_diffusion_core

#endif  // NAV2_DIFFUSION_CORE__PATH_HPP_
