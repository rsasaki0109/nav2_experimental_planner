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
Render Nav2 Planner Battle replays as GIFs from ``battle_data.json``.

Honest frames only — every pose and path comes from ``battle_trace`` (real plugins).

Usage::

    python3 tools/battle_gif_demo.py
    # writes docs/battle_race.gif, docs/battle_duel.gif, docs/battle_maze.gif
"""

from __future__ import annotations

import json
import os

import imageio.v2 as imageio
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(HERE, 'nav2_planner_battle', 'battle_data.json')
DOCS = os.path.join(HERE, '..', 'docs')

COLORS = [
    '#5ad1ff', '#ff5d6c', '#37e09a', '#ffd34d', '#c08bff', '#ff9f43',
    '#52e0e0', '#ff7bd5', '#9be15d', '#7aa2ff', '#ff6fae', '#d6e04d',
]

BG = '#0a1330'
GRID = '#16224a'
WALL = '#33406f'
WALL_EDGE = '#5566a8'


def _load():
    with open(DATA_PATH, encoding='utf-8') as f:
        return json.load(f)


def _setup_ax(arena, title):
    fig, ax = plt.subplots(figsize=(5.6, 5.6), dpi=100)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, arena['w'])
    ax.set_ylim(0, arena['h'])
    ax.set_aspect('equal')
    ax.set_title(title, color='#e8ecff', fontsize=11, pad=8)
    ax.tick_params(colors='#8b97c4', labelsize=7)
    for spine in ax.spines.values():
        spine.set_color('#25305c')
    for i in range(int(arena['w']) + 1):
        ax.axhline(i, color=GRID, lw=0.4, zorder=0)
        ax.axvline(i, color=GRID, lw=0.4, zorder=0)
    return fig, ax


def _draw_obstacles(ax, obstacles):
    for r in obstacles:
        ax.add_patch(Rectangle(
            (r['x'], r['y']), r['w'], r['h'],
            facecolor=WALL, edgecolor=WALL_EDGE, lw=0.8, zorder=1))


def _draw_start_goal(ax, start, goal):
    ax.plot(start[0], start[1], 'o', color='#37e09a', ms=8, zorder=5)
    ax.plot(goal[0], goal[1], 's', color='#ffd34d', ms=9, zorder=5)


def _maze_grid(ax, name, arena):
    n, step = (4, 1.5) if 'easy' in name.lower() else (8, 0.75)
    if 'micro mouse' not in name.lower():
        return
    for i in range(n + 1):
        c = i * step
        ax.axhline(c, color='#1e2d5a', lw=0.5, zorder=0)
        ax.axvline(c, color='#1e2d5a', lw=0.5, zorder=0)


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
    for t in range(0, n_frames, stride):
        fig, ax = _setup_ax(arena, f"Mode A · Race — {sc['name']}")
        _maze_grid(ax, sc['name'], arena)
        _draw_obstacles(ax, sc.get('obstacles', []))
        _draw_start_goal(ax, sc['start'], sc['goal'])
        for i, f in enumerate(fighters):
            col = COLORS[i % len(COLORS)]
            idx = min(t, len(f['path']) - 1)
            xs = [p[0] for p in f['path'][: idx + 1]]
            ys = [p[1] for p in f['path'][: idx + 1]]
            ax.plot(xs, ys, '-', color=col, lw=1.8, alpha=0.65, zorder=2)
            p = f['path'][idx]
            ax.plot(p[0], p[1], 'o', color=col, ms=6, zorder=4)
        images.append(_fig_to_array(fig))
        plt.close(fig)
    imageio.mimsave(out_path, images, duration=0.09, loop=0)
    print(f'wrote {out_path} ({len(images)} frames)')


def render_mode_b_duel(data, sc_idx, out_path, max_fighters=8, stride=3):
    sc = data['modeB']['scenarios'][sc_idx]
    arena = data['arena']
    ok = [f for f in sc['fighters'] if f.get('success') and f.get('path')]
    ok.sort(key=lambda f: f['length'])
    ok = ok[:max_fighters]
    n_frames = 45
    images = []
    for t in range(0, n_frames, stride):
        frac = min(1.0, t / max(1, n_frames - 1))
        fig, ax = _setup_ax(arena, f"Mode B · Duel — {sc['name']}")
        _maze_grid(ax, sc['name'], arena)
        _draw_obstacles(ax, sc.get('obstacles', []))
        _draw_start_goal(ax, sc['start'], sc['goal'])
        for i, f in enumerate(ok):
            col = COLORS[i % len(COLORS)]
            upto = max(1, int(frac * len(f['path'])))
            xs = [p[0] for p in f['path'][:upto]]
            ys = [p[1] for p in f['path'][:upto]]
            ax.plot(xs, ys, '-', color=col, lw=2.0, alpha=0.9, zorder=2)
            ax.plot(xs[-1], ys[-1], 'o', color=col, ms=4, zorder=4)
        images.append(_fig_to_array(fig))
        plt.close(fig)
    imageio.mimsave(out_path, images, duration=0.1, loop=0)
    print(f'wrote {out_path} ({len(images)} frames)')


def main():
    data = _load()
    os.makedirs(DOCS, exist_ok=True)
    # frontal: threading (5) vs learned (2) vs transformer (3)
    render_mode_a_race(
        data, sc_idx=1,
        out_path=os.path.join(DOCS, 'battle_race.gif'),
        fighter_idx=[2, 3, 5],
    )
    # micro mouse easy: ND (1) vs threading (5)
    render_mode_a_race(
        data, sc_idx=4,
        out_path=os.path.join(DOCS, 'battle_maze.gif'),
        fighter_idx=[1, 5],
    )
    # slalom duel: top planners by path length
    render_mode_b_duel(
        data, sc_idx=6,
        out_path=os.path.join(DOCS, 'battle_duel.gif'),
    )


if __name__ == '__main__':
    main()
