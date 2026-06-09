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
In-launch mission node for the headless Gazebo closed-loop benchmark.

Runs *inside* the simulation launch (so it shares the launch's DDS graph and does
not depend on external-process discovery, which is unreliable in restricted
sandboxes — see docs/simulation.md section 10.5). It waits for the Nav2
``navigate_to_pose`` action, then drives the robot through a **sequence of named
legs** (a mission course) with whatever FollowPath controller the params file
loaded, records the executed odometry per leg, and writes a single Markdown
**leaderboard** (one row per leg + a summary). The result file — not a ROS topic
— is the artifact, so the run is verifiable without joining the DDS graph.

A course mirrors the offline ``planner_benchmark`` sweep but closed-loop: each leg
is one ``NavigateToPose`` goal sent from wherever the previous leg ended, so the
same world exercises several goals in one bring-up. Pass legs via the ``missions``
string array (``"label|x|y|yaw|timeout"`` each); with none given it falls back to
the single ``goal_x`` / ``goal_y`` / ``goal_yaw`` goal for backward compatibility.

The leg-spec parsing, metric aggregation, and leaderboard formatting are pure
(no ROS) and unit-tested in ``test/test_sim_mission.py``; only the driving loop
needs a live Nav2 + Gazebo.
"""

from collections import namedtuple
import math
import time


# --- Pure helpers (no ROS; unit-tested in test/test_sim_mission.py) -----------

# One leg of a mission course: a goal pose plus a per-leg timeout.
Mission = namedtuple('Mission', ['label', 'x', 'y', 'yaw', 'timeout'])

# The measured outcome of one leg.
MissionOutcome = namedtuple(
    'MissionOutcome',
    ['label', 'reached', 'wall_time', 'path_len', 'samples', 'final_dist', 'note'])


def parse_missions(specs, default_timeout):
    """
    Parse ``"label|x|y[|yaw[|timeout]]"`` strings into a list of Mission.

    ``label`` may contain spaces; ``yaw`` and ``timeout`` are optional (yaw
    defaults to 0.0, timeout to ``default_timeout``). Blank entries are skipped.
    Raises ValueError on a malformed entry so a typo fails loudly rather than
    silently dropping a leg.
    """
    missions = []
    for spec in specs:
        spec = spec.strip()
        if not spec:
            continue
        parts = spec.split('|')
        if len(parts) < 3:
            raise ValueError(
                "mission spec '{}' needs at least 'label|x|y'".format(spec))
        label = parts[0].strip()
        try:
            x = float(parts[1])
            y = float(parts[2])
            yaw = float(parts[3]) if len(parts) > 3 and parts[3].strip() else 0.0
            timeout = (float(parts[4]) if len(parts) > 4 and parts[4].strip()
                       else float(default_timeout))
        except ValueError as exc:
            raise ValueError(
                "mission spec '{}' has a non-numeric field: {}".format(spec, exc))
        missions.append(Mission(label or 'leg', x, y, yaw, timeout))
    return missions


# Named course presets: a course name -> a list of "label|x|y[|yaw[|timeout]]" legs.
# Goal poses are in the map frame and tuned for the open `tb3_sandbox` world (TB3
# waffle spawns near x=-2.0, y=-0.5). These exercise sustained closed-loop nav
# (out-and-back, a patrol loop); obstacle courses that mirror the off-line
# planner_benchmark gaps/slalom need SDF walls in the world (future work) — see
# docs/simulation.md section 10.5.
COURSES = {
    'default': ['goal|0.0|-0.5|0.0|120'],
    'there_and_back': [
        'out|0.0|-0.5|0.0|120',
        'back|-2.0|-0.5|3.14159|120',
    ],
    'patrol': [
        'east|0.0|-0.5|0.0|120',
        'north|0.0|1.0|1.5708|120',
        'west|-2.0|1.0|3.14159|120',
        'home|-2.0|-0.5|-1.5708|120',
    ],
}


def course_to_missions(name, default_timeout):
    """
    Expand a named course preset into a list of Mission.

    ``name`` must be a key of ``COURSES`` (or empty -> []). Raises ValueError on an
    unknown name so a typo fails loudly with the list of valid courses.
    """
    if not name or not name.strip():
        return []
    key = name.strip()
    if key not in COURSES:
        raise ValueError(
            "unknown course '{}'; valid: {}".format(key, sorted(COURSES)))
    return parse_missions(COURSES[key], default_timeout)


def format_leaderboard(outcomes, frame_id, header_note):
    """Render a list of MissionOutcome as a Markdown leaderboard string."""
    reached = sum(1 for o in outcomes if o.reached)
    total_len = sum(o.path_len for o in outcomes)
    total_time = sum(o.wall_time for o in outcomes)
    lines = [
        '# Gazebo closed-loop mission result',
        '',
        '> {}'.format(header_note),
        '',
        '| Leg | Reached goal | Wall time [s] | Path length [m] | '
        'Odom samples | Final dist to goal [m] | Note |',
        '|---|:-:|--:|--:|--:|--:|---|',
    ]
    for o in outcomes:
        lines.append(
            '| {} | {} | {:.1f} | {:.2f} | {} | {:.3f} | {} |'.format(
                o.label, 'yes' if o.reached else 'no', o.wall_time, o.path_len,
                o.samples, o.final_dist, o.note))
    lines += [
        '',
        '**Summary:** {}/{} legs reached · total path {:.2f} m · '
        'total wall time {:.1f} s · frame `{}`.'.format(
            reached, len(outcomes), total_len, total_time, frame_id),
        '',
        '- Path length is the executed odometry distance; "reached" is the Nav2 '
        'action result (goal checker in the map frame).',
        '- Each leg is one NavigateToPose goal sent from where the previous leg '
        'ended; see [simulation.md](simulation.md) section 10.5.',
    ]
    return '\n'.join(lines) + '\n'


def yaw_to_quaternion(yaw):
    """Planar yaw [rad] -> (z, w) of a geometry_msgs/Quaternion."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


# --- ROS node (driving loop; needs a live Nav2 + Gazebo) ----------------------

def _run_node():
    """Construct and run the SimMission node (imports ROS lazily)."""
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import Quaternion
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    import rclpy
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.action import ActionClient
    from rclpy.node import Node

    def _float_param(node, name, default):
        """Declare a float param; launch may pass an integer (``180`` not ``180.0``)."""
        node.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True))
        return float(node.get_parameter(name).value)

    class SimMission(Node):
        """Drive a sequence of NavigateToPose legs, record odom, write a report."""

        def __init__(self):
            super().__init__('sim_mission')
            self.declare_parameter('goal_x', 0.0)
            self.declare_parameter('goal_y', -0.5)
            self.declare_parameter('goal_yaw', 0.0)
            self.declare_parameter('missions', [''])
            self.declare_parameter('course', '')
            self.declare_parameter('frame_id', 'map')
            self.timeout_sec = _float_param(self, 'timeout_sec', 120.0)
            self.server_wait_sec = _float_param(self, 'server_wait_sec', 60.0)
            self.declare_parameter('odom_topic', '/odom')
            self.declare_parameter('stop_on_failure', False)
            self.declare_parameter('label', 'Diffusion (Mode A) in Gazebo')
            self.declare_parameter('results_file', '/tmp/sim_mission_result.md')

            self.frame_id = self.get_parameter('frame_id').value
            self.stop_on_failure = self.get_parameter('stop_on_failure').value
            self.results_file = self.get_parameter('results_file').value

            # Precedence: explicit `missions` > named `course` preset > single goal.
            specs = [s for s in self.get_parameter('missions').value if s.strip()]
            self.missions = parse_missions(specs, self.timeout_sec)
            if not self.missions:
                self.missions = course_to_missions(
                    self.get_parameter('course').value, self.timeout_sec)
            if not self.missions:
                # Backward-compatible single-goal mission.
                self.missions = [Mission(
                    self.get_parameter('label').value,
                    float(self.get_parameter('goal_x').value),
                    float(self.get_parameter('goal_y').value),
                    float(self.get_parameter('goal_yaw').value),
                    float(self.timeout_sec))]

            self.path_len = 0.0
            self.steps = 0
            self.last_xy = None
            self.cur_xy = None
            self.sub = self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value, self.on_odom, 10)
            self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        def on_odom(self, msg):
            p = msg.pose.pose.position
            xy = (p.x, p.y)
            if self.last_xy is not None:
                self.path_len += math.hypot(
                    xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            self.last_xy = xy
            self.cur_xy = xy
            self.steps += 1

        def _wait_for_odom(self, timeout):
            deadline = time.time() + timeout
            while rclpy.ok() and self.cur_xy is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
            return self.cur_xy is not None

        def _wait_for_lifecycle(self, node_name, timeout):
            from lifecycle_msgs.msg import State
            from lifecycle_msgs.srv import GetState

            client = self.create_client(GetState, '{}/get_state'.format(node_name))
            if not client.wait_for_service(timeout_sec=min(60.0, timeout)):
                return False
            deadline = time.time() + timeout
            while rclpy.ok() and time.time() < deadline:
                req = GetState.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                result = future.result()
                if result and result.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    return True
                rclpy.spin_once(self, timeout_sec=0.5)
            return False

        def _wait_for_localization(self, timeout):
            """Wait until AMCL lifecycle is active (map loaded, ready for goals)."""
            return self._wait_for_lifecycle('amcl', timeout)

        def _drive_leg(self, mission):
            """Send one goal, block to completion/timeout, return MissionOutcome."""
            len0, steps0 = self.path_len, self.steps
            wall_start = time.time()

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = self.frame_id
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = mission.x
            goal.pose.pose.position.y = mission.y
            qz, qw = yaw_to_quaternion(mission.yaw)
            goal.pose.pose.orientation = Quaternion(z=qz, w=qw)

            self.get_logger().info('leg "{}" -> goal ({:.2f}, {:.2f})'.format(
                mission.label, mission.x, mission.y))
            send_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                return self._outcome(mission, False, 'goal rejected',
                                     wall_start, len0, steps0)

            result_future = handle.get_result_async()
            deadline = wall_start + mission.timeout
            while rclpy.ok() and not result_future.done():
                rclpy.spin_once(self, timeout_sec=0.2)
                if time.time() > deadline:
                    handle.cancel_goal_async()
                    rclpy.spin_once(self, timeout_sec=0.5)
                    return self._outcome(mission, False, 'timeout',
                                         wall_start, len0, steps0)

            status = result_future.result().status
            reached = status == GoalStatus.STATUS_SUCCEEDED
            note = 'succeeded' if reached else 'action status {}'.format(status)
            return self._outcome(mission, reached, note, wall_start, len0, steps0)

        def _outcome(self, mission, reached, note, wall_start, len0, steps0):
            final = self.cur_xy or (float('nan'), float('nan'))
            return MissionOutcome(
                label=mission.label,
                reached=reached,
                wall_time=time.time() - wall_start,
                path_len=self.path_len - len0,
                samples=self.steps - steps0,
                final_dist=math.hypot(final[0] - mission.x, final[1] - mission.y),
                note=note)

        def run(self):
            """Drive every leg in order, then write the leaderboard."""
            if not self.client.wait_for_server(timeout_sec=self.server_wait_sec):
                return self._write([], 'navigate_to_pose action server unavailable')
            if not self._wait_for_odom(self.server_wait_sec):
                return self._write([], 'no odometry received')
            if not self._wait_for_localization(self.server_wait_sec):
                return self._write([], 'amcl lifecycle inactive')
            if not self._wait_for_lifecycle('bt_navigator', self.server_wait_sec):
                return self._write([], 'bt_navigator lifecycle inactive')

            outcomes = []
            for mission in self.missions:
                outcome = self._drive_leg(mission)
                outcomes.append(outcome)
                self.get_logger().info('leg "{}" reached={} note={}'.format(
                    mission.label, outcome.reached, outcome.note))
                if self.stop_on_failure and not outcome.reached:
                    self.get_logger().warn('stop_on_failure: aborting remaining legs')
                    break
            self._write(outcomes, 'ok')

        def _write(self, outcomes, status_note):
            note = ('Auto-generated by `sim_mission` (in-launch). A course of '
                    'NavigateToPose legs driven by the FollowPath controller in a '
                    'headless TB3 Gazebo sim. See [simulation.md](simulation.md) '
                    'section 10.5.')
            if not outcomes:
                # Pre-flight failure: emit a one-row table that records why.
                outcomes = [MissionOutcome(
                    self.missions[0].label if self.missions else 'leg',
                    False, 0.0, 0.0, 0, float('nan'), status_note)]
            text = format_leaderboard(outcomes, self.frame_id, note)
            try:
                with open(self.results_file, 'w') as handle:
                    handle.write(text)
                self.get_logger().info('wrote result to ' + self.results_file)
            except OSError as exc:
                self.get_logger().error('failed to write result: {}'.format(exc))
            reached = sum(1 for o in outcomes if o.reached)
            self.get_logger().info('MISSION DONE {}/{} legs reached'.format(
                reached, len(outcomes)))

    rclpy.init()
    node = SimMission()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    _run_node()


if __name__ == '__main__':
    main()
