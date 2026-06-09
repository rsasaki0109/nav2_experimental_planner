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

#include "nav2_diffusion_controller/egocentric_patch.hpp"

#include <cmath>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"

namespace nav2_diffusion_controller
{

std::vector<float> cropEgocentricPatch(
  const nav2_costmap_2d::Costmap2D & costmap,
  double robot_x, double robot_y, double robot_yaw,
  int size, double patch_resolution)
{
  std::vector<float> patch(size > 0 ? static_cast<std::size_t>(size) * size : 0, 0.0f);
  if (size <= 0) {
    return patch;
  }

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double center = (size - 1) / 2.0;

  for (int r = 0; r < size; ++r) {
    const double forward = (center - r) * patch_resolution;   // +x base frame
    for (int c = 0; c < size; ++c) {
      const double left = (center - c) * patch_resolution;    // +y base frame
      const double wx = robot_x + forward * cos_yaw - left * sin_yaw;
      const double wy = robot_y + forward * sin_yaw + left * cos_yaw;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (costmap.worldToMap(wx, wy, mx, my)) {
        const unsigned char cost = costmap.getCost(mx, my);
        if (cost != nav2_costmap_2d::NO_INFORMATION) {
          patch[static_cast<std::size_t>(r) * size + c] = cost / 254.0f;
        }
      }
    }
  }
  return patch;
}

}  // namespace nav2_diffusion_controller
