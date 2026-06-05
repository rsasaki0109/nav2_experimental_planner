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

#include "nav2_diffusion_core/path.hpp"

#include <cmath>

namespace nav2_diffusion_core
{

double pathLength(const PathCandidate & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.points.size(); ++i) {
    const double dx = path.points[i].x - path.points[i - 1].x;
    const double dy = path.points[i].y - path.points[i - 1].y;
    length += std::hypot(dx, dy);
  }
  return length;
}

}  // namespace nav2_diffusion_core
