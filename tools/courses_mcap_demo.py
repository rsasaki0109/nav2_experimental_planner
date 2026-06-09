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
Export nav2_diffusion_sim course routes to MCAP for RViz / Lichtblick recording.

Writes ``docs/sim_courses.mcap`` — same topic names as battle MCAPs so
``record_rviz_gif.py`` can capture it.
"""

from __future__ import annotations

import heapq
import math
import os
import sys

from mcap_ros2.writer import Writer

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, '..', 'docs')
sys.path.insert(0, os.path.join(HERE, '..', 'generative', 'nav2_diffusion_sim'))

from nav2_diffusion_sim import gen_courses  # noqa: E402

# Reuse battle MCAP schema helpers.
from battle_mcap_demo import (  # noqa: E402
    RES, _ROOTS, _header, _msgdef, _path_msg, _pose, _quat,
)

ORDER = ['centred', 'gap', 'slalom', 'micro_mouse_easy', 'micro_mouse_hard']
ROBOT_RADIUS = 0.22
STEPS = 26
HOLD = 6


def _costmap_from_walls(extent, walls, ns):
    xmin, xmax, ymin, ymax = extent
    w = int(round((xmax - xmin) / RES))
    h = int(round((ymax - ymin) / RES))
    data = [0] * (w * h)
    for cx, cy, sx, sy in walls:
        x0 = max(0, int((cx - sx / 2 - xmin) / RES))
        x1 = min(w, int((cx + sx / 2 - xmin) / RES) + 1)
        y0 = max(0, int((cy - sy / 2 - ymin) / RES))
        y1 = min(h, int((cy + sy / 2 - ymin) / RES) + 1)
        for row in range(y0, y1):
            for col in range(x0, x1):
                data[row * w + col] = 100
    return {
        'header': _header(ns),
        'info': {
            'map_load_time': {'sec': 0, 'nanosec': 0},
            'resolution': RES, 'width': w, 'height': h,
            'origin': _pose(xmin, ymin),
        },
        'data': data,
    }


def _route(name):
    spec = gen_courses.COURSE_SPECS[name]
    xmin, xmax, ymin, ymax = spec['extent']
    cols = int(round((xmax - xmin) / RES))
    rows = int(round((ymax - ymin) / RES))
    blocked = [[False] * cols for _ in range(rows)]

    def mark(cx, cy, sx, sy):
        x0 = max(0, int((cx - sx / 2 - xmin) / RES))
        x1 = min(cols, int((cx + sx / 2 - xmin) / RES) + 1)
        y0 = max(0, int((cy - sy / 2 - ymin) / RES))
        y1 = min(rows, int((cy + sy / 2 - ymin) / RES) + 1)
        for row in range(y0, y1):
            for col in range(x0, x1):
                blocked[row][col] = True

    for wall in gen_courses.all_walls(name):
        mark(*wall)
    sx, sy, _ = spec['start']
    gx, gy = spec['goals'][0][1], spec['goals'][0][2]
    sc = (int((sx - xmin) / RES), int((sy - ymin) / RES))
    gc = (int((gx - xmin) / RES), int((gy - ymin) / RES))

    def heuristic(c):
        return math.hypot(c[0] - gc[0], c[1] - gc[1])

    open_set = [(heuristic(sc), 0.0, sc)]
    came = {}
    gscore = {sc: 0.0}
    while open_set:
        _, g, cur = heapq.heappop(open_set)
        if cur == gc:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return [(xmin + (c + 0.5) * RES, ymin + (r + 0.5) * RES)
                    for c, r in path]
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (cur[0] + dc, cur[1] + dr)
            if nb[0] < 0 or nb[1] < 0 or nb[0] >= cols or nb[1] >= rows:
                continue
            if blocked[nb[1]][nb[0]]:
                continue
            ng = g + 1.0
            if ng < gscore.get(nb, 1e9):
                gscore[nb] = ng
                came[nb] = cur
                heapq.heappush(open_set, (ng + heuristic(nb), ng, nb))
    return []


def _yaw_at(route, k):
    if k + 1 < len(route):
        dx = route[k + 1][0] - route[k][0]
        dy = route[k + 1][1] - route[k][1]
    elif k > 0:
        dx = route[k][0] - route[k - 1][0]
        dy = route[k][1] - route[k - 1][1]
    else:
        return 0.0
    return math.atan2(dy, dx)


def main():
    out = os.path.join(DOCS, 'sim_courses.mcap')
    schemas = {}
    dt_ns = 100_000_000
    frame_i = 0
    with open(out, 'wb') as fh:
        w = Writer(fh)
        for dt in _ROOTS:
            schemas[dt] = w.register_msgdef(dt, _msgdef(dt))
        for name in ORDER:
            spec = gen_courses.COURSE_SPECS[name]
            route = _route(name)
            if not route:
                raise RuntimeError('no route for {}'.format(name))
            walls = list(gen_courses.all_walls(name))
            goal = spec['goals'][0]
            idxs = [int(x) for x in
                    __import__('numpy').linspace(0, len(route) - 1, STEPS)]
            for k in idxs:
                ns = frame_i * dt_ns
                w.write_message(
                    '/battle/costmap', schemas['nav_msgs/msg/OccupancyGrid'],
                    _costmap_from_walls(spec['extent'], walls, ns), ns, ns)
                pts = [[p[0], p[1], _yaw_at(route, i)]
                       for i, p in enumerate(route[: k + 1])]
                w.write_message(
                    '/battle/path_0', schemas['nav_msgs/msg/Path'],
                    _path_msg(pts, ns), ns, ns)
                for i in range(1, 8):
                    w.write_message(
                        '/battle/path_{}'.format(i),
                        schemas['nav_msgs/msg/Path'],
                        _path_msg([], ns), ns, ns)
                w.write_message(
                    '/goal_pose', schemas['geometry_msgs/msg/PoseStamped'],
                    {'header': _header(ns),
                     'pose': _pose(goal[1], goal[2])}, ns, ns)
                frame_i += 1
            for _ in range(HOLD):
                ns = frame_i * dt_ns
                w.write_message(
                    '/battle/costmap', schemas['nav_msgs/msg/OccupancyGrid'],
                    _costmap_from_walls(spec['extent'], walls, ns), ns, ns)
                pts = [[p[0], p[1], _yaw_at(route, i)]
                       for i, p in enumerate(route)]
                w.write_message(
                    '/battle/path_0', schemas['nav_msgs/msg/Path'],
                    _path_msg(pts, ns), ns, ns)
                w.write_message(
                    '/goal_pose', schemas['geometry_msgs/msg/PoseStamped'],
                    {'header': _header(ns),
                     'pose': _pose(goal[1], goal[2])}, ns, ns)
                frame_i += 1
        w.finish()
    print('wrote {} ({} frames)'.format(out, frame_i))


if __name__ == '__main__':
    main()
