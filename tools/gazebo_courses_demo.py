#!/usr/bin/env python3
# Copyright 2026 Nav2PlannerBattle contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Render the closed-loop obstacle courses as an RViz / Lichtblick-style GIF.

Grid A* routes on the generated occupancy maps (same geometry as gz-sim worlds).
For optional 3D gz-sim footage see ``tools/gazebo_gif_demo.py``.

Usage::

    PYTHONPATH=generative/nav2_diffusion_sim python3 tools/gazebo_courses_demo.py
    # writes docs/sim_courses.gif
"""

import heapq
import math
import os

import imageio.v2 as imageio

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

import numpy as np  # noqa: E402

from nav2_diffusion_sim import gen_courses  # noqa: E402

RES = 0.05
ROBOT_RADIUS = 0.22  # TB3 waffle footprint radius (approx)
ORDER = ['centred', 'gap', 'slalom', 'micro_mouse_easy', 'micro_mouse_hard']

# RViz / Lichtblick palette (tools/mcap_view_gif.py)
BG = '#15151f'
GRID = '#2a2a3a'
OBST = '#e0544e'
PATH_DONE = '#3fb950'
PATH_TODO = '#2f81f7'
GOAL = '#f0c000'
START = '#9aa7b2'
TITLE = '#cfd3dc'
SUB = '#7a8190'


def _heading_at(route, k):
    if k + 1 < len(route):
        dx = route[k + 1, 0] - route[k, 0]
        dy = route[k + 1, 1] - route[k, 1]
    elif k > 0:
        dx = route[k, 0] - route[k - 1, 0]
        dy = route[k, 1] - route[k - 1, 1]
    else:
        return 0.0
    return math.atan2(dy, dx)


def _robot_wedge(ax, x, y, yaw, color, scale=0.2):
    tip = (x + scale * math.cos(yaw), y + scale * math.sin(yaw))
    back, wing = 0.55 * scale, 0.38 * scale
    left = (
        x - back * math.cos(yaw) + wing * math.cos(yaw + math.pi / 2),
        y - back * math.sin(yaw) + wing * math.sin(yaw + math.pi / 2),
    )
    right = (
        x - back * math.cos(yaw) + wing * math.cos(yaw - math.pi / 2),
        y - back * math.sin(yaw) + wing * math.sin(yaw - math.pi / 2),
    )
    ax.add_patch(Polygon([tip, left, right], closed=True,
                         facecolor=color, edgecolor='white', lw=0.6, zorder=5))


def _occupancy(name):
    """Build an inflated boolean occupancy grid (True = blocked) for a course."""
    xmin, xmax, ymin, ymax = gen_courses.COURSE_SPECS[name]['extent']
    w = int(round((xmax - xmin) / RES))
    h = int(round((ymax - ymin) / RES))
    grid = np.zeros((h, w), dtype=bool)
    inflate = int(round(ROBOT_RADIUS / RES))
    for cx, cy, sx, sy in gen_courses.all_walls(name):
        # Wall bounds inflated by the robot radius, clamped to the grid.
        x0 = int((cx - sx / 2 - ROBOT_RADIUS - xmin) / RES)
        x1 = int((cx + sx / 2 + ROBOT_RADIUS - xmin) / RES)
        y0 = int((cy - sy / 2 - ROBOT_RADIUS - ymin) / RES)
        y1 = int((cy + sy / 2 + ROBOT_RADIUS - ymin) / RES)
        grid[max(0, y0):min(h, y1 + 1), max(0, x0):min(w, x1 + 1)] = True
    return grid, (xmin, xmax, ymin, ymax), inflate


def _world_to_cell(x, y, extent):
    xmin, _, ymin, _ = extent
    return int((x - xmin) / RES), int((y - ymin) / RES)


def _astar(grid, start, goal):
    """8-connected grid A*; returns a list of (col, row) cells or []."""
    h, w = grid.shape
    sx, sy = start
    gx, gy = goal

    def heur(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    open_q = [(heur(start, goal), 0.0, start)]
    came = {}
    cost = {start: 0.0}
    while open_q:
        _, g, cur = heapq.heappop(open_q)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for dx, dy in nbrs:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < w and 0 <= ny < h) or grid[ny, nx]:
                continue
            step = 1.4142 if dx and dy else 1.0
            ng = g + step
            if ng < cost.get((nx, ny), 1e18):
                cost[(nx, ny)] = ng
                came[(nx, ny)] = cur
                heapq.heappush(open_q, (ng + heur((nx, ny), goal), ng, (nx, ny)))
    return []


def _route(name):
    """Return the world-frame route (Nx2) through a course."""
    grid, extent, _ = _occupancy(name)
    spec = gen_courses.COURSE_SPECS[name]
    start = _world_to_cell(spec['start'][0], spec['start'][1], extent)
    g = spec['goals'][0]
    goal = _world_to_cell(g[1], g[2], extent)
    cells = _astar(grid, start, goal)
    xmin, _, ymin, _ = extent
    return np.array([[xmin + (c + 0.5) * RES, ymin + (r + 0.5) * RES]
                     for c, r in cells])


def _draw(ax, name, route, k):
    """Draw one course frame — RViz / Lichtblick style with heading wedge."""
    ax.clear()
    spec = gen_courses.COURSE_SPECS[name]
    xmin, xmax, ymin, ymax = spec['extent']
    ax.set_facecolor(BG)
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color(GRID)
    # Costmap-style wall cells.
    res = 0.06
    for cx, cy, sx, sy in gen_courses.all_walls(name):
        x0, x1 = cx - sx / 2, cx + sx / 2
        y0, y1 = cy - sy / 2, cy + sy / 2
        xs = np.arange(x0, x1, res)
        ys = np.arange(y0, y1, res)
        gx, gy = np.meshgrid(xs, ys)
        ax.scatter(gx.ravel(), gy.ravel(), s=12, c=OBST, marker='s',
                   alpha=0.88, edgecolors='none', zorder=1)
    if len(route):
        ax.plot(route[:k + 1, 0], route[:k + 1, 1], '-', color=PATH_DONE,
                lw=2.8, zorder=3)
        ax.plot(route[k:, 0], route[k:, 1], '--', color=PATH_TODO,
                lw=1.6, alpha=0.65, zorder=2)
        rx, ry = route[k]
        _robot_wedge(ax, rx, ry, _heading_at(route, k), PATH_DONE)
    sx_, sy_, _ = spec['start']
    g = spec['goals'][0]
    ax.plot(sx_, sy_, 'o', color=START, ms=9, zorder=4)
    ax.plot(g[1], g[2], '*', color=GOAL, ms=18, zorder=4)
    ax.set_title('RViz-style · {}'.format(name), color=TITLE, fontsize=11,
                 loc='left', pad=8)
    ax.text(0.0, 1.02, 'nav2_diffusion_sim course · grid A* route',
            transform=ax.transAxes, color=SUB, fontsize=8)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, '..', 'docs', 'sim_courses.gif')
    routes = {n: _route(n) for n in ORDER}
    for n in ORDER:
        if not len(routes[n]):
            raise RuntimeError("no route found for course '{}'".format(n))

    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    fig.patch.set_facecolor(BG)
    frames = []
    steps = 26
    for name in ORDER:
        route = routes[name]
        idx = np.linspace(0, len(route) - 1, steps).astype(int)
        for k in idx:
            _draw(ax, name, route, int(k))
            fig.canvas.draw()
            buf = np.asarray(fig.canvas.buffer_rgba())[..., :3]
            frames.append(buf.copy())
        # Hold the final frame a moment before the next course.
        for _ in range(6):
            frames.append(frames[-1])
    imageio.mimsave(out, frames, duration=0.08, loop=0)
    print('wrote {} ({} frames)'.format(os.path.normpath(out), len(frames)))


if __name__ == '__main__':
    main()
