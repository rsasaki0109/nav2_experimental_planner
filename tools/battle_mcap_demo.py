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
Export Nav2 Planner Battle scenarios to MCAP for Lichtblick / Foxglove.

Topics (map frame, same names ``record_rviz_gif.py`` publishes live):
  /battle/costmap          nav_msgs/OccupancyGrid
  /battle/path_{i}         nav_msgs/Path  (truncated per frame)
  /battle/markers          visualization_msgs/MarkerArray (goal + robot arrows)
  /tf_static               map -> base_link
  /goal_pose               geometry_msgs/PoseStamped

Usage::

    python3 tools/battle_mcap_demo.py
    # writes docs/battle_race.mcap, battle_maze.mcap, battle_duel.mcap
"""

from __future__ import annotations

import json
import math
import os
import struct

from mcap_ros2.writer import Writer

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, 'nav2_planner_battle', 'battle_data.json')
DOCS = os.path.join(HERE, '..', 'docs')
RES = 0.05

_JOBS = [
    ('battle_race.mcap', 'modeA', 1, None),
    ('battle_maze.mcap', 'modeA', 4, None),
    ('battle_duel.mcap', 'modeB', 6, None),
]

_DEPS = {
    'builtin_interfaces/Time': 'int32 sec\nuint32 nanosec\n',
    'std_msgs/Header': 'builtin_interfaces/Time stamp\nstring frame_id\n',
    'geometry_msgs/Point': 'float64 x\nfloat64 y\nfloat64 z\n',
    'geometry_msgs/Quaternion': 'float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n',
    'geometry_msgs/Pose':
        'geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n',
    'geometry_msgs/PoseStamped':
        'std_msgs/Header header\ngeometry_msgs/Pose pose\n',
    'geometry_msgs/Vector3': 'float64 x\nfloat64 y\nfloat64 z\n',
    'geometry_msgs/Transform':
        'geometry_msgs/Vector3 translation\ngeometry_msgs/Quaternion rotation\n',
    'geometry_msgs/TransformStamped':
        'std_msgs/Header header\nstring child_frame_id\n'
        'geometry_msgs/Transform transform\n',
    'nav_msgs/MapMetaData':
        'builtin_interfaces/Time map_load_time\nfloat32 resolution\n'
        'uint32 width\nuint32 height\ngeometry_msgs/Pose origin\n',
    'nav_msgs/msg/OccupancyGrid':
        'std_msgs/Header header\nnav_msgs/MapMetaData info\nint8[] data\n',
    'nav_msgs/msg/Path':
        'std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses\n',
    'visualization_msgs/msg/Marker':
        'std_msgs/Header header\nstring ns\nint32 id\nint32 type\nint32 action\n'
        'geometry_msgs/Pose pose\ngeometry_msgs/Vector3 scale\n'
        'std_msgs/ColorRGBA color\nbuiltin_interfaces/Duration lifetime\n'
        'bool frame_locked\ngeometry_msgs/Point[] points\n'
        'std_msgs/ColorRGBA[] colors\nstring text\nstring mesh_resource\n'
        'bool mesh_use_embedded_materials\n',
    'visualization_msgs/msg/MarkerArray':
        'visualization_msgs/Marker[] markers\n',
    'tf2_msgs/msg/TFMessage':
        'geometry_msgs/TransformStamped[] transforms\n',
    'std_msgs/ColorRGBA': 'float32 r\nfloat32 g\nfloat32 b\nfloat32 a\n',
    'builtin_interfaces/Duration': 'int32 sec\nuint32 nanosec\n',
}

_ROOTS = {
    'nav_msgs/msg/OccupancyGrid': (
        'std_msgs/Header header\nnav_msgs/MapMetaData info\nint8[] data\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'nav_msgs/MapMetaData',
         'geometry_msgs/Pose', 'geometry_msgs/Point', 'geometry_msgs/Quaternion']),
    'nav_msgs/msg/Path': (
        'std_msgs/Header header\ngeometry_msgs/PoseStamped[] poses\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/PoseStamped',
         'geometry_msgs/Pose', 'geometry_msgs/Point', 'geometry_msgs/Quaternion']),
    'visualization_msgs/msg/MarkerArray': (
        'visualization_msgs/Marker[] markers\n',
        ['visualization_msgs/msg/Marker', 'std_msgs/Header', 'builtin_interfaces/Time',
         'geometry_msgs/Pose', 'geometry_msgs/Point', 'geometry_msgs/Quaternion',
         'geometry_msgs/Vector3', 'std_msgs/ColorRGBA', 'builtin_interfaces/Duration']),
    'geometry_msgs/msg/PoseStamped': (
        'std_msgs/Header header\ngeometry_msgs/Pose pose\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/Pose',
         'geometry_msgs/Point', 'geometry_msgs/Quaternion']),
    'tf2_msgs/msg/TFMessage': (
        'geometry_msgs/TransformStamped[] transforms\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/TransformStamped',
         'geometry_msgs/Transform', 'geometry_msgs/Vector3', 'geometry_msgs/Quaternion']),
}


def _msgdef(datatype):
    root, deps = _ROOTS[datatype]
    out = root
    for dep in deps:
        out += '=' * 80 + '\nMSG: {}\n{}'.format(dep, _DEPS[dep])
    return out


def _header(ns):
    return {'stamp': {'sec': ns // 1_000_000_000, 'nanosec': ns % 1_000_000_000},
            'frame_id': 'map'}


def _quat(yaw):
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}


def _pose(x, y, yaw=0.0):
    return {'position': {'x': float(x), 'y': float(y), 'z': 0.0},
            'orientation': _quat(yaw)}


def _costmap(arena, obstacles, ns):
    w = int(round(arena['w'] / RES))
    h = int(round(arena['h'] / RES))
    data = [0] * (w * h)
    for r in obstacles:
        x0 = max(0, int((r['x']) / RES))
        x1 = min(w, int((r['x'] + r['w']) / RES) + 1)
        y0 = max(0, int((r['y']) / RES))
        y1 = min(h, int((r['y'] + r['h']) / RES) + 1)
        for row in range(y0, y1):
            for col in range(x0, x1):
                data[row * w + col] = 100
    return {
        'header': _header(ns),
        'info': {
            'map_load_time': {'sec': 0, 'nanosec': 0},
            'resolution': RES, 'width': w, 'height': h,
            'origin': _pose(0.0, 0.0),
        },
        'data': data,
    }


def _path_msg(points, ns):
    poses = []
    for p in points:
        yaw = p[2] if len(p) > 2 else 0.0
        poses.append({'header': _header(ns), 'pose': _pose(p[0], p[1], yaw)})
    return {'header': _header(ns), 'poses': poses}


def _markers(fighters, frame_idx, goal, ns):
    markers = []
    markers.append({
        'header': _header(ns), 'ns': 'goal', 'id': 0, 'type': 2, 'action': 0,
        'pose': _pose(goal[0], goal[1]), 'scale': {'x': 0.35, 'y': 0.35, 'z': 0.2},
        'color': {'r': 0.95, 'g': 0.75, 'b': 0.1, 'a': 1.0},
        'lifetime': {'sec': 0, 'nanosec': 0}, 'frame_locked': False,
        'points': [], 'colors': [], 'text': '', 'mesh_resource': '',
        'mesh_use_embedded_materials': False,
    })
    colors = [
        (0.25, 0.72, 0.31), (0.18, 0.51, 0.97), (0.35, 0.82, 1.0),
        (1.0, 0.36, 0.42), (1.0, 0.83, 0.3), (0.75, 0.52, 0.99),
        (1.0, 0.62, 0.26), (0.32, 0.82, 0.88),
    ]
    for i, f in enumerate(fighters):
        idx = min(frame_idx, len(f['path']) - 1)
        p = f['path'][idx]
        yaw = p[2] if len(p) > 2 else 0.0
        cr, cg, cb = colors[i % len(colors)]
        markers.append({
            'header': _header(ns), 'ns': 'robot', 'id': i + 1, 'type': 0,
            'action': 0, 'pose': _pose(p[0], p[1], yaw),
            'scale': {'x': 0.45, 'y': 0.12, 'z': 0.08},
            'color': {'r': cr, 'g': cg, 'b': cb, 'a': 1.0},
            'lifetime': {'sec': 0, 'nanosec': 0}, 'frame_locked': False,
            'points': [], 'colors': [], 'text': f.get('label', ''),
            'mesh_resource': '', 'mesh_use_embedded_materials': False,
        })
    return {'markers': markers}


def _fighters(sc, mode, pick):
    fs = sc['fighters']
    if mode == 'modeB':
        ok = [f for f in fs if f.get('success') and f.get('path')]
        ok.sort(key=lambda f: f['length'])
        return ok[:8]
    if pick is not None:
        return [fs[i] for i in pick]
    return fs


def _frame_count(fighters, mode):
    if mode == 'modeB':
        return 48
    return max(len(f['path']) for f in fighters)


def export_job(data, fname, mode_key, sc_idx, pick, stride=2):
    sc = data[mode_key]['scenarios'][sc_idx]
    arena = data['arena']
    fighters = _fighters(sc, mode_key, pick)
    n = _frame_count(fighters, mode_key)
    out = os.path.join(DOCS, fname)
    schemas = {}
    dt_ns = 100_000_000
    with open(out, 'wb') as fh:
        w = Writer(fh)
        for dt in _ROOTS:
            schemas[dt] = w.register_msgdef(dt, _msgdef(dt))
        w.write_message('/tf_static', schemas['tf2_msgs/msg/TFMessage'], {
            'transforms': [{
                'header': _header(0), 'child_frame_id': 'base_link',
                'transform': {
                    'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                },
            }]}, 0, 0)
        frame_i = 0
        for t in range(0, n, stride):
            ns = frame_i * dt_ns
            w.write_message('/battle/costmap', schemas['nav_msgs/msg/OccupancyGrid'],
                            _costmap(arena, sc.get('obstacles', []), ns), ns, ns)
            if mode_key == 'modeB':
                frac = min(1.0, t / max(1, n - 1))
                for i, f in enumerate(fighters):
                    upto = max(1, int(frac * len(f['path'])))
                    pts = [[p[0], p[1], p[2] if len(p) > 2 else 0.0]
                           for p in f['path'][:upto]]
                    w.write_message(
                        '/battle/path_{}'.format(i),
                        schemas['nav_msgs/msg/Path'], _path_msg(pts, ns), ns, ns)
                # markers at tip
                tips = []
                for f in fighters:
                    upto = max(1, int(frac * len(f['path'])))
                    p = f['path'][upto - 1]
                    tips.append({'path': [p], 'label': f.get('label', '')})
                w.write_message('/battle/markers',
                                schemas['visualization_msgs/msg/MarkerArray'],
                                _markers(tips, 0, sc['goal'], ns), ns, ns)
            else:
                for i, f in enumerate(fighters):
                    idx = min(t, len(f['path']) - 1)
                    pts = [[p[0], p[1], p[2] if len(p) > 2 else 0.0]
                           for p in f['path'][: idx + 1]]
                    w.write_message(
                        '/battle/path_{}'.format(i),
                        schemas['nav_msgs/msg/Path'], _path_msg(pts, ns), ns, ns)
                w.write_message('/battle/markers',
                                schemas['visualization_msgs/msg/MarkerArray'],
                                _markers(fighters, t, sc['goal'], ns), ns, ns)
            w.write_message('/goal_pose', schemas['geometry_msgs/msg/PoseStamped'],
                            {'header': _header(ns),
                             'pose': _pose(sc['goal'][0], sc['goal'][1])}, ns, ns)
            frame_i += 1
        w.finish()
    print('wrote {} ({} frames)'.format(out, frame_i))


def main():
    with open(DATA, encoding='utf-8') as fh:
        data = json.load(fh)
    os.makedirs(DOCS, exist_ok=True)
    for fname, mode_key, sc_idx, pick in _JOBS:
        export_job(data, fname, mode_key, sc_idx, pick)


if __name__ == '__main__':
    main()
