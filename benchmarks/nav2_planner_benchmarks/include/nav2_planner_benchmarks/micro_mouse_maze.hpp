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

#ifndef NAV2_PLANNER_BENCHMARKS__MICRO_MOUSE_MAZE_HPP_
#define NAV2_PLANNER_BENCHMARKS__MICRO_MOUSE_MAZE_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace nav2_planner_benchmarks
{

/// Axis-aligned wall segment for a micro-mouse-style orthogonal maze.
struct WallSeg
{
  enum class Axis { HORIZONTAL, VERTICAL };
  Axis axis;
  double coord;  // world y (H) or x (V) [m]
  double a0, a1;  // span along the perpendicular axis [m]
};

struct Rect
{
  double x, y, w, h;  // lower-left corner + size, world metres
};

struct MicroMouseLayout
{
  std::string name;          // e.g. "micro mouse easy"
  std::string description;
  double start_x, start_y;
  double goal_x, goal_y;
  double cell_size;          // metres per grid cell (for UI overlay)
  int grid_cells;
  std::vector<WallSeg> walls;
};

inline constexpr int kMicroMouseHalfCells = 1;  // wall thickness ≈ 0.15 m at 0.05 m res

inline const std::vector<WallSeg> & microMouseEasyWalls()
{
  // 4×4 perfect maze (1.5 m cells), backtracker seed 7 — training / exploration run.
  static const std::vector<WallSeg> walls = {
    {WallSeg::Axis::HORIZONTAL, 1.5000, 0.0000, 1.5000},
    {WallSeg::Axis::HORIZONTAL, 3.0000, 1.5000, 4.5000},
    {WallSeg::Axis::HORIZONTAL, 6.0000, 0.0000, 6.0000},
    {WallSeg::Axis::VERTICAL, 1.5000, 3.0000, 4.5000},
    {WallSeg::Axis::VERTICAL, 3.0000, 0.0000, 3.0000},
    {WallSeg::Axis::VERTICAL, 3.0000, 4.5000, 6.0000},
    {WallSeg::Axis::VERTICAL, 4.5000, 1.5000, 4.5000},
    {WallSeg::Axis::VERTICAL, 6.0000, 0.0000, 6.0000},
  };
  return walls;
}

inline const std::vector<WallSeg> & microMouseHardWalls()
{
  // 8×8 perfect maze (0.75 m cells), backtracker seed 42 — contest / speed run.
  static const std::vector<WallSeg> walls = {
    {WallSeg::Axis::HORIZONTAL, 0.7500, 1.5000, 3.0000},
    {WallSeg::Axis::HORIZONTAL, 0.7500, 4.5000, 5.2500},
    {WallSeg::Axis::HORIZONTAL, 1.5000, 1.5000, 2.2500},
    {WallSeg::Axis::HORIZONTAL, 1.5000, 5.2500, 6.0000},
    {WallSeg::Axis::HORIZONTAL, 2.2500, 0.7500, 1.5000},
    {WallSeg::Axis::HORIZONTAL, 2.2500, 4.5000, 5.2500},
    {WallSeg::Axis::HORIZONTAL, 3.0000, 1.5000, 2.2500},
    {WallSeg::Axis::HORIZONTAL, 3.0000, 3.0000, 5.2500},
    {WallSeg::Axis::HORIZONTAL, 3.7500, 3.0000, 3.7500},
    {WallSeg::Axis::HORIZONTAL, 3.7500, 4.5000, 5.2500},
    {WallSeg::Axis::HORIZONTAL, 4.5000, 1.5000, 4.5000},
    {WallSeg::Axis::HORIZONTAL, 4.5000, 5.2500, 6.0000},
    {WallSeg::Axis::HORIZONTAL, 5.2500, 0.7500, 1.5000},
    {WallSeg::Axis::HORIZONTAL, 5.2500, 4.5000, 5.2500},
    {WallSeg::Axis::HORIZONTAL, 6.0000, 0.0000, 6.0000},
    {WallSeg::Axis::VERTICAL, 0.7500, 0.0000, 5.2500},
    {WallSeg::Axis::VERTICAL, 1.5000, 0.7500, 1.5000},
    {WallSeg::Axis::VERTICAL, 1.5000, 3.0000, 4.5000},
    {WallSeg::Axis::VERTICAL, 2.2500, 1.5000, 3.7500},
    {WallSeg::Axis::VERTICAL, 2.2500, 4.5000, 6.0000},
    {WallSeg::Axis::VERTICAL, 3.0000, 0.7500, 2.2500},
    {WallSeg::Axis::VERTICAL, 3.0000, 3.0000, 3.7500},
    {WallSeg::Axis::VERTICAL, 3.0000, 5.2500, 6.0000},
    {WallSeg::Axis::VERTICAL, 3.7500, 0.0000, 3.0000},
    {WallSeg::Axis::VERTICAL, 3.7500, 4.5000, 5.2500},
    {WallSeg::Axis::VERTICAL, 4.5000, 0.7500, 2.2500},
    {WallSeg::Axis::VERTICAL, 4.5000, 3.7500, 4.5000},
    {WallSeg::Axis::VERTICAL, 5.2500, 2.2500, 3.0000},
    {WallSeg::Axis::VERTICAL, 5.2500, 4.5000, 5.2500},
    {WallSeg::Axis::VERTICAL, 6.0000, 0.0000, 6.0000},
  };
  return walls;
}

inline MicroMouseLayout microMouseEasyLayout()
{
  return MicroMouseLayout{
    "micro mouse easy",
    "Micro-mouse easy: 4×4 orthogonal grid (1.5 m cells), SW start, centre goal — "
    "training / exploration run (fewer branches than hard)",
    0.75, 0.75, 2.25, 2.25, 1.5, 4, std::vector<WallSeg>(microMouseEasyWalls().begin(), microMouseEasyWalls().end())};
}

inline MicroMouseLayout microMouseHardLayout()
{
  return MicroMouseLayout{
    "micro mouse hard",
    "Micro-mouse hard: 8×8 orthogonal grid (0.75 m cells), SW start, centre goal — "
    "contest / speed run (All Japan MicroMouse inspired)",
    0.5, 0.5, 3.375, 3.375, 0.75, 8,
    std::vector<WallSeg>(microMouseHardWalls().begin(), microMouseHardWalls().end())};
}

inline const std::vector<MicroMouseLayout> & microMouseLayouts()
{
  static const std::vector<MicroMouseLayout> layouts = {
    microMouseEasyLayout(),
    microMouseHardLayout(),
  };
  return layouts;
}

inline bool isMicroMouseScenario(const std::string & name)
{
  return name.find("micro mouse") != std::string::npos;
}

inline const MicroMouseLayout * findMicroMouseLayout(const std::string & name)
{
  for (const auto & layout : microMouseLayouts()) {
    if (layout.name == name) {
      return &layout;
    }
  }
  // Legacy battle data used plain "micro mouse" for the hard course.
  if (name == "micro mouse") {
    return &microMouseLayouts()[1];
  }
  return nullptr;
}

inline Rect hwallRect(double wy, double x0, double x1, int half_cells, double res)
{
  const double th = (2 * half_cells + 1) * res;
  return Rect{x0, wy - th / 2.0, x1 - x0, th};
}

inline Rect vwallRect(double wx, double y0, double y1, int half_cells, double res)
{
  const double th = (2 * half_cells + 1) * res;
  return Rect{wx - th / 2.0, y0, th, y1 - y0};
}

inline void markHWall(
  nav2_costmap_2d::Costmap2D * costmap, double wy, double x0, double x1, int half_cells)
{
  const double res = costmap->getResolution();
  for (double x = x0; x <= x1; x += res) {
    unsigned int cx = 0;
    unsigned int cy = 0;
    if (!costmap->worldToMap(x, wy, cx, cy)) {
      continue;
    }
    for (int dy = -half_cells; dy <= half_cells; ++dy) {
      costmap->setCost(cx, cy + dy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

inline void markVWall(
  nav2_costmap_2d::Costmap2D * costmap, double wx, double y0, double y1, int half_cells)
{
  const double res = costmap->getResolution();
  for (double y = y0; y <= y1; y += res) {
    unsigned int cx = 0;
    unsigned int cy = 0;
    if (!costmap->worldToMap(wx, y, cx, cy)) {
      continue;
    }
    for (int dx = -half_cells; dx <= half_cells; ++dx) {
      costmap->setCost(cx + dx, cy, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

inline void markWalls(
  nav2_costmap_2d::Costmap2D * costmap, const std::vector<WallSeg> & walls,
  int half_cells = kMicroMouseHalfCells)
{
  for (const auto & w : walls) {
    if (w.axis == WallSeg::Axis::HORIZONTAL) {
      markHWall(costmap, w.coord, w.a0, w.a1, half_cells);
    } else {
      markVWall(costmap, w.coord, w.a0, w.a1, half_cells);
    }
  }
}

inline void markMicroMouseScenario(
  nav2_costmap_2d::Costmap2D * costmap, const std::string & scenario_name,
  int half_cells = kMicroMouseHalfCells)
{
  const auto * layout = findMicroMouseLayout(scenario_name);
  if (layout != nullptr) {
    markWalls(costmap, layout->walls, half_cells);
  }
}

inline std::vector<Rect> obstacleRectsFromWalls(
  const std::vector<WallSeg> & walls, int half_cells = kMicroMouseHalfCells, double res = 0.05)
{
  std::vector<Rect> rects;
  rects.reserve(walls.size());
  for (const auto & w : walls) {
    if (w.axis == WallSeg::Axis::HORIZONTAL) {
      rects.push_back(hwallRect(w.coord, w.a0, w.a1, half_cells, res));
    } else {
      rects.push_back(vwallRect(w.coord, w.a0, w.a1, half_cells, res));
    }
  }
  return rects;
}

inline std::vector<Rect> microMouseObstacleRects(
  const std::string & scenario_name, int half_cells = kMicroMouseHalfCells, double res = 0.05)
{
  const auto * layout = findMicroMouseLayout(scenario_name);
  if (layout == nullptr) {
    return {};
  }
  return obstacleRectsFromWalls(layout->walls, half_cells, res);
}

// Back-compat aliases (hard maze).
inline const std::vector<WallSeg> & microMouseWalls()
{
  return microMouseHardWalls();
}

inline void markMicroMouseMaze(
  nav2_costmap_2d::Costmap2D * costmap, int half_cells = kMicroMouseHalfCells)
{
  markWalls(costmap, microMouseHardWalls(), half_cells);
}

inline constexpr double kMicroMouseStartX = 0.5;
inline constexpr double kMicroMouseStartY = 0.5;
inline constexpr double kMicroMouseGoalX = 3.375;
inline constexpr double kMicroMouseGoalY = 3.375;

inline std::string microMouseDescription()
{
  return microMouseHardLayout().description;
}

}  // namespace nav2_planner_benchmarks

#endif  // NAV2_PLANNER_BENCHMARKS__MICRO_MOUSE_MAZE_HPP_
