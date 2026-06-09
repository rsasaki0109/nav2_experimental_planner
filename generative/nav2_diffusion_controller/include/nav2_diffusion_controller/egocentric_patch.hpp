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

#ifndef NAV2_DIFFUSION_CONTROLLER__EGOCENTRIC_PATCH_HPP_
#define NAV2_DIFFUSION_CONTROLLER__EGOCENTRIC_PATCH_HPP_

#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_diffusion_controller
{

/// Crop a heading-aligned egocentric costmap patch, row-major, normalized [0, 1]
/// (1 = lethal). The patch is rotated by robot_yaw so it is consistent
/// regardless of heading: row 0 is ahead of the robot, increasing row goes
/// behind; increasing column goes to the robot's right. Cells outside the
/// costmap or with no information map to 0.
///
/// @param size patch side length in cells; @param patch_resolution metres/cell.
std::vector<float> cropEgocentricPatch(
  const nav2_costmap_2d::Costmap2D & costmap,
  double robot_x, double robot_y, double robot_yaw,
  int size, double patch_resolution);

}  // namespace nav2_diffusion_controller

#endif  // NAV2_DIFFUSION_CONTROLLER__EGOCENTRIC_PATCH_HPP_
