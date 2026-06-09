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
Render Nav2 Planner Battle README GIFs in a Lichtblick / RViz-style view.

Every pose and path comes from ``battle_trace`` (real plugins). The look matches
``tools/mcap_view_gif.py`` — dark viewer background, costmap-red obstacles,
coloured plan traces, and **heading triangles** so motion direction reads clearly.

Usage::

    python3 tools/battle_gif_demo.py
    # writes docs/battle_race.gif, docs/battle_duel.gif, docs/battle_maze.gif
"""

from __future__ import annotations

import json
import math
import os

import imageio.v2 as imageio
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(HERE, 'nav2_planner_battle', 'battle_data.json')
DOCS = os.path.join(HERE, '..', 'docs')

# Lichtblick / Foxglove palette (see tools/mcap_view_gif.py)
BG = '#15151f'
GRID = '#2a2a3a'
OBST = '#e0544e'
GOAL = '#f0c000'
START = '#9aa7b2'
TITLE = '#cfd3dc'
SUB = '#7a8190'

PATH_COLORS = [
    '#3fb950', '#2f81f7', '#5ad1ff', '#ff5d6c', '#ffd34d', '#c08bff',
    '#ff9f43', '#52e0e0', '#ff7bd5', '#9be15d', '#7aa2ff', '#d6e04d',
]


def _load():
    with open(DATA_PATH, encoding='utf-8') as f:
        return json.load(f)


def _heading_at(path, idx):
    if idx + 1 < len(path):
        dx = path[idx + 1][0] - path[idx][0]
        dy = path[idx + 1][1] - path[idx][1]
    elif idx > 0:
        dx = path[idx][0] - path[idx - 1][0]
        dy = path[idx][1] - path[idx - 1][1]
    else:
        return 0.0
    if abs(dx) + abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)


def _robot_wedge(ax, x, y, yaw, color, scale=0.22):
    """RViz-style pose arrow (points along path tangent)."""
    tip = (x + scale * math.cos(yaw), y + scale * math.sin(yaw))
    back = 0.55 * scale
    wing = 0.38 * scale
    left = (
        x - back * math.cos(yaw) + wing * math.cos(yaw + math.pi / 2),
        y - back * math.sin(yaw) + wing * math.sin(yaw + math.pi / 2),
    )
    right = (
        x - back * math.cos(yaw) + wing * math.cos(yaw - math.pi / 2),
        y - back * math.sin(yaw) + wing * math.sin(yaw - math.pi / 2),
    )
    ax.add_patch(Polygon([tip, left, right], closed=True,
                         facecolor=color, edgecolor='white', lw=0.6, zorder=6))


def _setup_ax(arena, title, subtitle):
    fig, ax = plt.subplots(figsize=(6.4, 4.8), dpi=100)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, arena['w'])
    ax.set_ylim(0, arena['h'])
    ax.set_aspect('equal')
    ax.set_title(title, color=TITLE, fontsize=11, pad=10, loc='left')
    ax.text(0.0, 1.02, subtitle, transform=ax.transAxes, color=SUB, fontsize=8)
    ax.tick_params(colors=SUB, labelsize=6)
    for spine in ax.spines.values():
        spine.set_color(GRID)
    step = max(1, int(max(arena['w'], arena['h']) / 6))
    for i in range(0, int(arena['w']) + 1, step):
        ax.axvline(i, color=GRID, lw=0.35, zorder=0)
    for i in range(0, int(arena['h']) + 1, step):
        ax.axhline(i, color=GRID, lw=0.35, zorder=0)
    return fig, ax


def _draw_costmap_obstacles(ax, obstacles, res=0.08):
    """Rasterise obstacle rects into costmap-style red cells."""
    for r in obstacles:
        xs = np.arange(r['x'], r['x'] + r['w'], res)
        ys = np.arange(r['y'], r['y'] + r['h'], res)
        gx, gy = np.meshgrid(xs, ys)
        ax.scatter(gx.ravel(), gy.ravel(), s=14, c=OBST, marker='s',
                   alpha=0.9, edgecolors='none', zorder=1)


def _maze_grid(ax, name):
    if 'micro mouse' not in name.lower():
        return
    n, step = (4, 1.5) if 'easy' in name.lower() else (8, 0.75)
    for i in range(n + 1):
        c = i * step
        ax.plot([c, c], [0, n * step], color=GRID, lw=0.4, zorder=0)
        ax.plot([0, n * step], [c, c], color=GRID, lw=0.4, zorder=0)


def _draw_start_goal(ax, start, goal):
    ax.plot(start[0], start[1], 'o', color=START, ms=9, zorder=5)
    ax.plot(goal[0], goal[1], '*', color=GOAL, ms=20, zorder=5)


def _fig_to_array(fig):
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())
    return buf[:, :, :3].copy()


def render_mode_a_race(data, sc_idx, out_path, fighter_idx=None, stride=2):
    sc = data['modeA']['scenarios'][sc_idx]
    arena = data['arena']
    fighters = sc['fighters']
    if fighter_idx is not None:
        fighters = [fighters[i] for i in fighter_idx]
    n_frames = max(len(f['path']) for f in fighters)
    images = []
    title = 'Mode A · Race — {}'.format(sc['name'])
    subtitle = 'Lichtblick-style view · battle_trace controllers'
    for t in range(0, n_frames, stride):
        fig, ax = _setup_ax(arena, title, subtitle)
        _maze_grid(ax, sc['name'])
        _draw_costmap_obstacles(ax, sc.get('obstacles', []))
        _draw_start_goal(ax, sc['start'], sc['goal'])
        for i, f in enumerate(fighters):
            col = PATH_COLORS[i % len(PATH_COLORS)]
            idx = min(t, len(f['path']) - 1)
            xs = [p[0] for p in f['path'][: idx + 1]]
            ys = [p[1] for p in f['path'][: idx + 1]]
            ax.plot(xs, ys, '-', color=col, lw=2.2, alpha=0.85, zorder=3)
            p = f['path'][idx]
            _robot_wedge(ax, p[0], p[1], _heading_at(f['path'], idx), col)
        images.append(_fig_to_array(fig))
        plt.close(fig)
    imageio.mimsave(out_path, images, duration=0.09, loop=0)
    print('wrote {} ({} frames)'.format(out_path, len(images)))


def render_mode_b_duel(data, sc_idx, out_path, max_fighters=8, stride=3):
    sc = data['modeB']['scenarios'][sc_idx]
    arena = data['arena']
    ok = [f for f in sc['fighters'] if f.get('success') and f.get('path')]
    ok.sort(key=lambda f: f['length'])
    ok = ok[:max_fighters]
    n_frames = 48
    images = []
    title = 'Mode B · Duel — {}'.format(sc['name'])
    subtitle = 'Lichtblick-style view · battle_trace global planners'
    for t in range(0, n_frames, stride):
        frac = min(1.0, t / max(1, n_frames - 1))
        fig, ax = _setup_ax(arena, title, subtitle)
        _maze_grid(ax, sc['name'])
        _draw_costmap_obstacles(ax, sc.get('obstacles', []))
        _draw_start_goal(ax, sc['start'], sc['goal'])
        for i, f in enumerate(ok):
            col = PATH_COLORS[i % len(PATH_COLORS)]
            upto = max(1, int(frac * len(f['path'])))
            xs = [p[0] for p in f['path'][:upto]]
            ys = [p[1] for p in f['path'][:upto]]
            ax.plot(xs, ys, '-', color=col, lw=2.4, alpha=0.9, zorder=3)
            if upto > 1:
                _robot_wedge(ax, xs[-1], ys[-1],
                             _heading_at(f['path'], upto - 1), col, scale=0.16)
        images.append(_fig_to_array(fig))
        plt.close(fig)
    imageio.mimsave(out_path, images, duration=0.1, loop=0)
    print('wrote {} ({} frames)'.format(out_path, len(images)))


def main():
    data = _load()
    os.makedirs(DOCS, exist_ok=True)
    render_mode_a_race(
        data, sc_idx=1,
        out_path=os.path.join(DOCS, 'battle_race.gif'),
        fighter_idx=[2, 3, 5],
    )
    render_mode_a_race(
        data, sc_idx=4,
        out_path=os.path.join(DOCS, 'battle_maze.gif'),
        fighter_idx=[1, 5],
    )
    render_mode_b_duel(
        data, sc_idx=6,
        out_path=os.path.join(DOCS, 'battle_duel.gif'),
    )


if __name__ == '__main__':
    main()
