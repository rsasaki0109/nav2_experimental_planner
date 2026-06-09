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
Render the demo MCAP's contents as a viewer-style (Lichtblick/Foxglove) GIF.

A GUI viewer can't be screen-recorded in this headless environment, so this draws
the *actual messages* in ``docs/mode_b_demo.mcap`` — the same file you open in the
Lichtblick / Foxglove 3D panel — in a dark, viewer-like top-down style: the
OccupancyGrid costmap, the rejected / safe candidate paths, the selected path, and
the goal, scrubbing through the recorded timeline. It is an honest render of the
MCAP content, not a screenshot of the viewer.

Usage::

    pip install mcap mcap-ros2-support imageio matplotlib
    python3 tools/mcap_view_gif.py     # reads docs/mode_b_demo.mcap, writes docs/mcap_view.gif
"""

import os
from collections import defaultdict

import imageio.v2 as imageio

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

import numpy as np  # noqa: E402

from mcap_ros2.reader import read_ros2_messages  # noqa: E402

BG = '#15151f'        # viewer dark background
GRID = '#2a2a3a'

HERE = os.path.dirname(os.path.abspath(__file__))
MCAP = os.path.join(HERE, '..', 'docs', 'mode_b_demo.mcap')
OUT = os.path.join(HERE, '..', 'docs', 'mcap_view.gif')


def _poses_xy(msg):
    return [(p.position.x, p.position.y) for p in msg.poses]


def _path_xy(msg):
    return [(p.pose.position.x, p.pose.position.y) for p in msg.poses]


def _grid_obstacles(g):
    """Return (xs, ys) world coords of occupied cells in an OccupancyGrid."""
    w, h, res = g.info.width, g.info.height, g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    data = np.asarray(g.data, dtype=np.int16).reshape(h, w)
    rows, cols = np.where(data >= 50)
    xs = ox + (cols + 0.5) * res
    ys = oy + (rows + 0.5) * res
    return xs, ys


def main():
    # Group messages by recorded timestamp (one frame per timeline tick).
    frames = defaultdict(dict)
    for m in read_ros2_messages(MCAP):
        frames[m.log_time_ns][m.channel.topic] = m.ros_msg
    times = sorted(frames)

    fig, ax = plt.subplots(figsize=(4.6, 4.4))
    fig.patch.set_facecolor(BG)
    images = []
    for i, t in enumerate(times):
        f = frames[t]
        ax.clear()
        ax.set_facecolor(BG)
        ax.set_xlim(-0.4, 5.6)
        ax.set_ylim(-2.7, 2.7)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color(GRID)

        if '/local_costmap' in f:
            xs, ys = _grid_obstacles(f['/local_costmap'])
            ax.scatter(xs, ys, s=10, c='#e0544e', marker='s', alpha=0.85,
                       edgecolors='none')
        if '/candidates_rejected' in f:
            pts = _poses_xy(f['/candidates_rejected'])
            if pts:
                a = np.array(pts)
                ax.scatter(a[:, 0], a[:, 1], s=5, c='#d9534f', alpha=0.35,
                           edgecolors='none')
        if '/candidates_safe' in f:
            pts = _poses_xy(f['/candidates_safe'])
            if pts:
                a = np.array(pts)
                ax.scatter(a[:, 0], a[:, 1], s=6, c='#2f81f7', alpha=0.6,
                           edgecolors='none')
        if '/path_best' in f:
            pts = _path_xy(f['/path_best'])
            if pts:
                a = np.array(pts)
                ax.plot(a[:, 0], a[:, 1], '-', color='#3fb950', lw=3.0, zorder=5)
        if '/goal_pose' in f:
            gp = f['/goal_pose'].pose.position
            ax.plot(gp.x, gp.y, '*', color='#f0c000', ms=18, zorder=6)
        ax.plot(0, 0, 'o', color='#9aa7b2', ms=8, zorder=6)   # start / robot

        ax.set_title('Lichtblick / Foxglove view of mode_b_demo.mcap',
                     color='#cfd3dc', fontsize=9)
        ax.text(0.02, 0.02,
                'frame {:02d}/{:02d}  •  /local_costmap /path_best /candidates_*'
                .format(i + 1, len(times)),
                transform=ax.transAxes, color='#7a8190', fontsize=6.5)
        fig.tight_layout(pad=0.4)
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[..., :3]
        images.append(buf.copy())
    imageio.mimsave(OUT, images, duration=0.12, loop=0)
    print('wrote {} ({} frames)'.format(os.path.normpath(OUT), len(images)))


if __name__ == '__main__':
    main()
