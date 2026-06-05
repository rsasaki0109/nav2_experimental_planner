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

#include "nav2_diffusion_core/trajectory.hpp"

#include <cmath>
#include <cstddef>

namespace nav2_diffusion_core
{

double pathLength(const Trajectory & trajectory)
{
  double length = 0.0;
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const double dx = trajectory.points[i].x - trajectory.points[i - 1].x;
    const double dy = trajectory.points[i].y - trajectory.points[i - 1].y;
    length += std::hypot(dx, dy);
  }
  return length;
}

double duration(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    return 0.0;
  }
  return trajectory.points.back().time - trajectory.points.front().time;
}

}  // namespace nav2_diffusion_core
