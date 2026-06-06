#!/usr/bin/env python3
# Copyright 2026 nav2_experimental_planner contributors
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
Record the real Mode B (global path) pipeline to an MCAP for Foxglove.

Runs the shipped generative ``PathFlowPlanner`` (the same proposer the GIF demo
uses), sweeps an obstacle across the corridor, and writes each frame's costmap +
proposed/validated/selected paths as standard ROS 2 messages into an MCAP file.
Open the result in Foxglove Studio (File → Open local file), add a 3D panel, and
scrub / play the timeline — the selected path (green) re-routes as the obstacle
moves. From there Foxglove can export a video.

This is a real recording of the model pipeline written to a portable file; it is
*not* a live Gazebo/ROS run (the dev sandbox blocks inter-process DDS and has no
display, so a Foxglove screen-capture cannot be produced here — the MCAP is the
honest, verifiable artifact you record from). See docs/visualization.md.

Topics: /local_costmap (OccupancyGrid), /path_best (Path),
/candidates_safe and /candidates_rejected (PoseArray), /goal_pose (PoseStamped).

Usage::

    pip install torch mcap mcap-ros2-support
    PYTHONPATH=generative/nav2_diffusion_training python3 tools/foxglove_mcap_demo.py
    # writes docs/mode_b_demo.mcap
"""

import math
import os

from mcap_ros2.writer import Writer

import numpy as np

import torch

from nav2_diffusion_training.path_planners import make_path_dataset, PathFlowPlanner

GOAL_D = 5.0
ROBOT_R = 0.18
OBST_X0, OBST_X1 = 2.0, 3.4
OBST_HALF_W = 0.30
RES = 0.1
X_MIN, X_MAX = -0.5, 5.5
Y_MIN, Y_MAX = -2.6, 2.6

# --- ROS 2 message definitions (concatenated .msg text for mcap_ros2) ----------
_DEPS = {
    'builtin_interfaces/Time': 'int32 sec\nuint32 nanosec\n',
    'std_msgs/Header': 'builtin_interfaces/Time stamp\nstring frame_id\n',
    'geometry_msgs/Point': 'float64 x\nfloat64 y\nfloat64 z\n',
    'geometry_msgs/Quaternion': 'float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n',
    'geometry_msgs/Pose':
        'geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n',
    'geometry_msgs/PoseStamped':
        'std_msgs/Header header\ngeometry_msgs/Pose pose\n',
    'nav_msgs/MapMetaData':
        'builtin_interfaces/Time map_load_time\nfloat32 resolution\n'
        'uint32 width\nuint32 height\ngeometry_msgs/Pose origin\n',
    'geometry_msgs/Vector3': 'float64 x\nfloat64 y\nfloat64 z\n',
    'geometry_msgs/Transform':
        'geometry_msgs/Vector3 translation\ngeometry_msgs/Quaternion rotation\n',
    'geometry_msgs/TransformStamped':
        'std_msgs/Header header\nstring child_frame_id\n'
        'geometry_msgs/Transform transform\n',
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
    'geometry_msgs/msg/PoseArray': (
        'std_msgs/Header header\ngeometry_msgs/Pose[] poses\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/Pose',
         'geometry_msgs/Point', 'geometry_msgs/Quaternion']),
    'geometry_msgs/msg/PoseStamped': (
        'std_msgs/Header header\ngeometry_msgs/Pose pose\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/Pose',
         'geometry_msgs/Point', 'geometry_msgs/Quaternion']),
    'tf2_msgs/msg/TFMessage': (
        'geometry_msgs/TransformStamped[] transforms\n',
        ['std_msgs/Header', 'builtin_interfaces/Time', 'geometry_msgs/TransformStamped',
         'geometry_msgs/Transform', 'geometry_msgs/Vector3',
         'geometry_msgs/Quaternion']),
}


def _msgdef(datatype):
    """Assemble the concatenated .msg definition text for a root datatype."""
    root, deps = _ROOTS[datatype]
    out = root
    for dep in deps:
        out += '=' * 80 + '\nMSG: {}\n{}'.format(dep, _DEPS[dep])
    return out


def _pose(x, y):
    return {'position': {'x': float(x), 'y': float(y), 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}


def _header(frame, ns):
    return {'stamp': {'sec': ns // 1_000_000_000, 'nanosec': ns % 1_000_000_000},
            'frame_id': frame}


def hits_obstacle(path, cy):
    for x, y in path:
        if OBST_X0 - ROBOT_R <= x <= OBST_X1 + ROBOT_R and \
                abs(y - cy) <= OBST_HALF_W + ROBOT_R:
            return True
    return False


def path_length(path):
    return sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
               for i in range(1, len(path)))


def _costmap(cy, ns):
    """OccupancyGrid with the obstacle rectangle marked lethal (100)."""
    w = int(round((X_MAX - X_MIN) / RES))
    h = int(round((Y_MAX - Y_MIN) / RES))
    data = [0] * (w * h)
    for row in range(h):
        y = Y_MIN + (row + 0.5) * RES
        for col in range(w):
            x = X_MIN + (col + 0.5) * RES
            if OBST_X0 <= x <= OBST_X1 and abs(y - cy) <= OBST_HALF_W:
                data[row * w + col] = 100
    return {
        'header': _header('map', ns),
        'info': {
            'map_load_time': {'sec': 0, 'nanosec': 0},
            'resolution': RES, 'width': w, 'height': h,
            'origin': _pose(X_MIN, Y_MIN),
        },
        'data': data,
    }


def train():
    """Train (or load) the shipped generative path model."""
    torch.manual_seed(0)
    cache = '/tmp/gifdata/mode_b_model.pt'
    model = PathFlowPlanner(steps=4)
    if os.path.exists(cache):
        model.load_state_dict(torch.load(cache))
        model.eval()
        return model
    ctx, tg = make_path_dataset(64)
    opt = torch.optim.Adam(model.parameters(), lr=0.012)
    for _ in range(220):
        opt.zero_grad()
        loss = model.flow_loss(ctx, tg)
        out = model(ctx[:8])
        jerk = out[:, :, 2:, :] - 2 * out[:, :, 1:-1, :] + out[:, :, :-2, :]
        (loss + 2.0 * (jerk ** 2).mean()).backward()
        opt.step()
    model.eval()
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    torch.save(model.state_dict(), cache)
    return model


def main():
    model = train()
    with torch.no_grad():
        cands = model(torch.tensor([[GOAL_D, 0.0]]))[0].numpy()  # [K, H, 2]
    paths = [[(float(p[0]), float(p[1])) for p in cands[k]] for k in range(len(cands))]

    centers = (list(np.linspace(1.2, 0.5, 6)) + list(np.linspace(-0.5, -1.2, 6)) +
               list(np.linspace(-1.2, -0.5, 6)) + list(np.linspace(0.5, 1.2, 6)))

    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, '..', 'docs', 'mode_b_demo.mcap')
    schemas = {}
    with open(out, 'wb') as fh:
        w = Writer(fh)
        for dt in _ROOTS:
            schemas[dt] = w.register_msgdef(dt, _msgdef(dt))
        dt_grid = 'nav_msgs/msg/OccupancyGrid'
        dt_path = 'nav_msgs/msg/Path'
        dt_pa = 'geometry_msgs/msg/PoseArray'
        dt_ps = 'geometry_msgs/msg/PoseStamped'
        dt_tf = 'tf2_msgs/msg/TFMessage'
        dt_ns = 100_000_000  # 0.1 s per frame
        # Static identity map->base_link transform (published once, latched) so a
        # viewer's 3D panel has a frame tree available at load — without /tf_static
        # the 3D panel shows "no frames" and renders nothing.
        w.write_message('/tf_static', schemas[dt_tf], {'transforms': [{
            'header': _header('map', 0), 'child_frame_id': 'base_link',
            'transform': {'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                          'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
        }]}, 0, 0)
        for i, cy in enumerate(centers):
            ns = i * dt_ns
            safe = [(k, p) for k, p in enumerate(paths) if not hits_obstacle(p, cy)]
            best_i = min(safe, key=lambda kp: path_length(kp[1]))[0] if safe else -1

            w.write_message('/local_costmap', schemas[dt_grid], _costmap(cy, ns), ns, ns)

            best_poses = ([{'header': _header('map', ns), 'pose': _pose(x, y)}
                           for x, y in paths[best_i]] if best_i >= 0 else [])
            w.write_message('/path_best', schemas[dt_path],
                            {'header': _header('map', ns), 'poses': best_poses}, ns, ns)

            safe_pts = [_pose(x, y) for k, p in safe if k != best_i for x, y in p]
            rej_pts = [_pose(x, y) for k, p in enumerate(paths)
                       if hits_obstacle(p, cy) for x, y in p]
            w.write_message('/candidates_safe', schemas[dt_pa],
                            {'header': _header('map', ns), 'poses': safe_pts}, ns, ns)
            w.write_message('/candidates_rejected', schemas[dt_pa],
                            {'header': _header('map', ns), 'poses': rej_pts}, ns, ns)
            w.write_message('/goal_pose', schemas[dt_ps],
                            {'header': _header('map', ns), 'pose': _pose(GOAL_D, 0.0)},
                            ns, ns)
        w.finish()
    print('wrote {} ({} frames)'.format(os.path.normpath(out), len(centers)))


if __name__ == '__main__':
    main()
