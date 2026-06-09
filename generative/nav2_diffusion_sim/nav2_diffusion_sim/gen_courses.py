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
Single-source course generator for the closed-loop Gazebo benchmark.

Each course is defined ONCE as a spec (map extent + start pose + goals + wall
boxes). From that one spec this module emits three mutually-consistent artifacts:

* a self-contained ``<course>.sdf.xacro`` Gazebo (gz-sim) world — physics /
  sensors / sun / ground plus the course walls, mirroring the stock
  ``tb3_sandbox`` plugin block so the simulated LiDAR renders;
* a matching occupancy map (``<course>.pgm`` + ``<course>.yaml``) so AMCL /
  the global costmap see the same walls the simulator does;
* the mission course (``"label|x|y|yaw|timeout"`` legs) for ``sim_mission.py``.

Generating all three from one spec means the world, the map, and the goals cannot
drift apart. The closed-loop run itself still needs a real ROS host (the sandbox
blocks inter-process DDS — see docs/simulation.md section 10.5); the geometry and
artifact well-formedness are unit-tested here.

The courses mirror the off-line ``planner_benchmark`` obstacle scenarios
(centred gap / off-centre gap / slalom) and the browser-battle **micro-mouse**
mazes (easy / hard — same wall layout as
``nav2_planner_benchmarks/micro_mouse_maze.hpp``) so the closed-loop stack can be
measured on the same shapes the proposal-stage benchmark and Nav2 Planner Battle use.
"""

import math
import os

# A wall is an axis-aligned box: (center_x, center_y, size_x, size_y) in metres.
# Height is fixed; z is derived so the box rests on the ground plane.
WALL_HEIGHT = 0.5
WALL_MARGIN = 0.15  # perimeter offset inside the map extent
MAZE_WALL_TH = 0.15  # matches micro_mouse_maze.hpp kMicroMouseHalfCells @ 0.05 m res


def _h_wall(coord, a0, a1, th=MAZE_WALL_TH):
    """Horizontal maze segment → axis-aligned box (matches battle_trace walls)."""
    return ((a0 + a1) / 2.0, coord, a1 - a0, th)


def _v_wall(coord, a0, a1, th=MAZE_WALL_TH):
    """Vertical maze segment → axis-aligned box."""
    return (coord, (a0 + a1) / 2.0, th, a1 - a0)


def _yaw_toward(sx, sy, gx, gy):
    return math.atan2(gy - sy, gx - sx)


# Interior walls only — perimeter is added by ``perimeter_walls`` (same as battle 6×6).
_MICRO_MOUSE_EASY_WALLS = [
    _h_wall(1.5, 0.0, 1.5), _h_wall(3.0, 1.5, 4.5), _h_wall(6.0, 0.0, 6.0),
    _v_wall(1.5, 3.0, 4.5), _v_wall(3.0, 0.0, 3.0), _v_wall(3.0, 4.5, 6.0),
    _v_wall(4.5, 1.5, 4.5), _v_wall(6.0, 0.0, 6.0),
]

_MICRO_MOUSE_HARD_WALLS = [
    _h_wall(0.75, 1.5, 3.0), _h_wall(0.75, 4.5, 5.25),
    _h_wall(1.5, 1.5, 2.25), _h_wall(1.5, 5.25, 6.0),
    _h_wall(2.25, 0.75, 1.5), _h_wall(2.25, 4.5, 5.25),
    _h_wall(3.0, 1.5, 2.25), _h_wall(3.0, 3.0, 5.25),
    _h_wall(3.75, 3.0, 3.75), _h_wall(3.75, 4.5, 5.25),
    _h_wall(4.5, 1.5, 4.5), _h_wall(4.5, 5.25, 6.0),
    _h_wall(5.25, 0.75, 1.5), _h_wall(5.25, 4.5, 5.25),
    _h_wall(6.0, 0.0, 6.0),
    _v_wall(0.75, 0.0, 5.25), _v_wall(1.5, 0.75, 1.5), _v_wall(1.5, 3.0, 4.5),
    _v_wall(2.25, 1.5, 3.75), _v_wall(2.25, 4.5, 6.0),
    _v_wall(3.0, 0.75, 2.25), _v_wall(3.0, 3.0, 3.75), _v_wall(3.0, 5.25, 6.0),
    _v_wall(3.75, 0.0, 3.0), _v_wall(3.75, 4.5, 5.25),
    _v_wall(4.5, 0.75, 2.25), _v_wall(4.5, 3.75, 4.5),
    _v_wall(5.25, 2.25, 3.0), _v_wall(5.25, 4.5, 5.25),
    _v_wall(6.0, 0.0, 6.0),
]

# Each course: map extent (xmin, xmax, ymin, ymax), start (x, y, yaw), goals
# (label, x, y, yaw, timeout), and interior wall boxes. A perimeter is added
# automatically so AMCL has features and the robot stays bounded — except for
# micro-mouse mazes, which match the open-arena battle layout (boundary segments
# are already in ``walls``).
COURSE_SPECS = {
    'centred': {
        'description': 'Wall with a gap on the straight line (dead ahead).',
        'extent': (-3.5, 3.5, -3.0, 3.0),
        'start': (-2.0, -0.5, 0.0),
        'goals': [('through', 2.0, -0.5, 0.0, 120.0)],
        # Gap y in [-1.0, 0.0] (centred on the y=-0.5 start->goal line).
        'walls': [
            (0.0, -2.0, 0.2, 2.0),   # below gap: y[-3,-1]
            (0.0, 1.5, 0.2, 3.0),    # above gap: y[0,3]
        ],
    },
    'gap': {
        'description': 'Wall with a gap ~2 m off the straight line (off-centre).',
        'extent': (-3.5, 3.5, -3.0, 3.0),
        'start': (-2.0, -0.5, 0.0),
        'goals': [('through', 2.0, -0.5, 0.0, 150.0)],
        # Gap y in [1.0, 2.0]; the y=-0.5 line is blocked, forcing a 2 m detour up.
        'walls': [
            (0.0, -1.0, 0.2, 4.0),   # below gap: y[-3,1]
            (0.0, 2.5, 0.2, 1.0),    # above gap: y[2,3]
        ],
    },
    'slalom': {
        'description': 'Two staggered walls (gap low then high): an S-shaped detour.',
        'extent': (-3.5, 3.5, -3.5, 3.5),
        'start': (-2.0, -0.5, 0.0),
        'goals': [('through', 2.5, 1.5, 0.0, 180.0)],
        'walls': [
            # Wall A at x=-0.5, gap y in [-2.0, -1.0] (robot must dip low).
            (-0.5, -2.75, 0.2, 1.5),  # y[-3.5,-2.0]
            (-0.5, 1.25, 0.2, 4.5),   # y[-1.0,3.5]
            # Wall B at x=1.0, gap y in [1.0, 2.0] (then swing high).
            (1.0, -1.25, 0.2, 4.5),   # y[-3.5,1.0]
            (1.0, 2.75, 0.2, 1.5),    # y[2.0,3.5]
        ],
    },
    'micro_mouse_easy': {
        'description': (
            'Micro-mouse easy: 4×4 grid (1.5 m cells), SW start, centre goal — '
            'matches Nav2 Planner Battle micro mouse easy / battle_trace.'),
        'extent': (0.0, 6.0, 0.0, 6.0),
        'start': (0.75, 0.75, _yaw_toward(0.75, 0.75, 2.25, 2.25)),
        'goals': [('centre', 2.25, 2.25, 0.0, 240.0)],
        'walls': _MICRO_MOUSE_EASY_WALLS,
        'perimeter': False,
    },
    'micro_mouse_hard': {
        'description': (
            'Micro-mouse hard: 8×8 grid (0.75 m cells), SW start, centre goal — '
            'matches Nav2 Planner Battle micro mouse hard / battle_trace.'),
        'extent': (0.0, 6.0, 0.0, 6.0),
        # Battle trace uses (0.5, 0.5) for a point robot; nudge inward for TB3 footprint.
        'start': (0.425, 0.425, _yaw_toward(0.425, 0.425, 3.375, 3.375)),
        'goals': [('centre', 3.375, 3.375, 0.0, 360.0)],
        'walls': _MICRO_MOUSE_HARD_WALLS,
        'perimeter': False,
    },
}


def perimeter_walls(extent, thickness=0.1):
    """Return four boxes forming a closed rectangle just inside ``extent``."""
    xmin, xmax, ymin, ymax = extent
    cx, cy = (xmin + xmax) / 2.0, (ymin + ymax) / 2.0
    w, h = xmax - xmin, ymax - ymin
    m = WALL_MARGIN
    return [
        (cx, ymin + m, w - 2 * m, thickness),   # bottom
        (cx, ymax - m, w - 2 * m, thickness),   # top
        (xmin + m, cy, thickness, h - 2 * m),   # left
        (xmax - m, cy, thickness, h - 2 * m),   # right
    ]


def all_walls(name):
    """Return interior + (optional) perimeter wall boxes for a course."""
    spec = COURSE_SPECS[name]
    walls = list(spec['walls'])
    if spec.get('perimeter', True):
        walls += perimeter_walls(spec['extent'])
    return walls


def point_in_any_wall(name, x, y, margin=0.0):
    """Return whether (x, y) lies in any wall box (optionally inflated by margin)."""
    for cx, cy, sx, sy in all_walls(name):
        if (abs(x - cx) <= sx / 2.0 + margin and
                abs(y - cy) <= sy / 2.0 + margin):
            return True
    return False


def course_missions(name):
    """Return the mission course as ``"label|x|y|yaw|timeout"`` strings."""
    return ['{}|{}|{}|{}|{}'.format(label, x, y, yaw, timeout)
            for label, x, y, yaw, timeout in COURSE_SPECS[name]['goals']]


def _wall_model_sdf(idx, box):
    """Render one wall box as a static SDF <model> block."""
    cx, cy, sx, sy = box
    return (
        '    <model name="wall_{idx}">\n'
        '      <static>true</static>\n'
        '      <pose>{cx} {cy} {hz} 0 0 0</pose>\n'
        '      <link name="link">\n'
        '        <collision name="collision">\n'
        '          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>\n'
        '        </collision>\n'
        '        <visual name="visual">\n'
        '          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>\n'
        '          <material><ambient>0.6 0.2 0.2 1</ambient>'
        '<diffuse>0.7 0.2 0.2 1</diffuse></material>\n'
        '        </visual>\n'
        '      </link>\n'
        '    </model>\n'
    ).format(idx=idx, cx=cx, cy=cy, hz=WALL_HEIGHT / 2.0,
             sx=sx, sy=sy, h=WALL_HEIGHT)


# gz-sim world preamble mirroring nav2_minimal_tb3_sim's tb3_sandbox plugin set
# (physics / user-commands / sensors / imu + sun + ground) so the LiDAR renders.
_WORLD_HEAD = """<?xml version="1.0"?>
<sdf version="1.6" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="headless" default="true"/>
  <world name="default">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" \
name="gz::sim::systems::UserCommands"/>
    <xacro:unless value="$(arg headless)">
      <plugin filename="gz-sim-scene-broadcaster-system" \
name="gz::sim::systems::SceneBroadcaster"/>
    </xacro:unless>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>1</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

_WORLD_TAIL = '  </world>\n</sdf>\n'


def world_sdf(name):
    """Render the full ``<course>.sdf.xacro`` world string."""
    walls = ''.join(_wall_model_sdf(i, b) for i, b in enumerate(all_walls(name)))
    return _WORLD_HEAD + walls + _WORLD_TAIL


def occupancy_map(name, resolution=0.05):
    """
    Render the occupancy grid for a course.

    Returns ``(pgm_bytes, yaml_str, (width, height))``. Free cells are 254
    (white), wall cells 0 (black); the PGM is binary P5 with row 0 at max-y so it
    matches the map_server origin convention (lower-left = extent min).
    """
    xmin, xmax, ymin, ymax = COURSE_SPECS[name]['extent']
    width = int(round((xmax - xmin) / resolution))
    height = int(round((ymax - ymin) / resolution))
    grid = bytearray([254]) * (width * height)
    for row in range(height):
        # Row 0 is the top of the image (max y).
        y = ymax - (row + 0.5) * resolution
        for col in range(width):
            x = xmin + (col + 0.5) * resolution
            if point_in_any_wall(name, x, y):
                grid[row * width + col] = 0
    header = 'P5\n{} {}\n255\n'.format(width, height).encode('ascii')
    pgm = header + bytes(grid)
    yaml = (
        'image: {name}.pgm\n'
        'mode: trinary\n'
        'resolution: {res}\n'
        'origin: [{xmin}, {ymin}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    ).format(name=name, res=resolution, xmin=xmin, ymin=ymin)
    return pgm, yaml, (width, height)


def main():
    """Write worlds/ and maps/ artifacts for every course."""
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    worlds_dir = os.path.join(pkg, 'worlds')
    maps_dir = os.path.join(pkg, 'maps')
    os.makedirs(worlds_dir, exist_ok=True)
    os.makedirs(maps_dir, exist_ok=True)
    for name in COURSE_SPECS:
        with open(os.path.join(worlds_dir, name + '.sdf.xacro'), 'w') as fh:
            fh.write(world_sdf(name))
        pgm, yaml, size = occupancy_map(name)
        with open(os.path.join(maps_dir, name + '.pgm'), 'wb') as fh:
            fh.write(pgm)
        with open(os.path.join(maps_dir, name + '.yaml'), 'w') as fh:
            fh.write(yaml)
        print('wrote course {} (map {}x{}): {}'.format(
            name, size[0], size[1], course_missions(name)))


if __name__ == '__main__':
    main()
