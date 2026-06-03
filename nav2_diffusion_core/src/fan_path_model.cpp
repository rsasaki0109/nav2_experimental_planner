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

#include "nav2_diffusion_core/fan_path_model.hpp"

#include <algorithm>
#include <cmath>

namespace nav2_diffusion_core
{

std::string FanPathModel::name() const
{
  return "fan_path";
}

std::vector<PathCandidate> FanPathModel::generate(const PathContext & context) const
{
  const int count = std::max(1, context.num_candidates);
  const int num_points = std::max(2, context.num_points);

  const double dx = context.goal_x - context.start_x;
  const double dy = context.goal_y - context.start_y;
  const double dist = std::hypot(dx, dy);

  // Unit vector perpendicular to the straight start->goal line (left normal).
  double px = 0.0;
  double py = 0.0;
  if (dist > 1e-6) {
    px = -dy / dist;
    py = dx / dist;
  }
  const double max_bow = max_bow_fraction_ * dist;

  std::vector<PathCandidate> candidates;
  candidates.reserve(count);
  for (int c = 0; c < count; ++c) {
    // Sweep the bow amplitude symmetrically; a single candidate is the straight
    // line, an odd count always includes amplitude 0 (straight) in the middle.
    double amp = 0.0;
    if (count > 1) {
      const double frac = static_cast<double>(c) / static_cast<double>(count - 1);
      amp = (-1.0 + 2.0 * frac) * max_bow;
    }

    PathCandidate candidate;
    candidate.points.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(num_points - 1);
      const double base_x = context.start_x + t * dx;
      const double base_y = context.start_y + t * dy;
      const double bow = amp * std::sin(M_PI * t);  // 0 at both endpoints
      candidate.points.push_back({base_x + bow * px, base_y + bow * py});
    }
    // Soft prior: prefer straighter (shorter) candidates. Not a safety signal.
    candidate.model_score = -std::abs(amp);
    candidates.push_back(candidate);
  }
  return candidates;
}

}  // namespace nav2_diffusion_core
