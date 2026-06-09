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
Rule-based expert track generators for bootstrapping training data.

These produce expert SE(2) tracks without a simulator (docs/training.md section
6.5, "rule-based ... initial prior"), so a synthetic imitation dataset can be
built and tested end to end: expert track -> build_samples -> save_jsonl.
"""

import math
from typing import List

from nav2_diffusion_training.dataset import TrackState


def _normalize_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.fmod(angle + math.pi, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def unicycle_to_goal(
    start: TrackState,
    goal_x: float,
    goal_y: float,
    speed: float = 0.3,
    max_omega: float = 1.0,
    heading_gain: float = 2.0,
    dt: float = 0.1,
    tolerance: float = 0.1,
    max_steps: int = 1000,
) -> List[TrackState]:
    """
    Drive a unicycle from ``start`` toward a goal, returning the executed track.

    The robot turns toward the goal (proportional heading control) while moving
    forward, slowing the forward speed when it is not yet facing the goal. The
    track ends when within ``tolerance`` of the goal or after ``max_steps``.
    """
    x, y, yaw, t = start.x, start.y, start.yaw, start.time
    track = [TrackState(time=t, x=x, y=y, yaw=yaw)]

    for _ in range(max_steps):
        if math.hypot(goal_x - x, goal_y - y) <= tolerance:
            break
        heading_error = _normalize_angle(math.atan2(goal_y - y, goal_x - x) - yaw)
        omega = max(-max_omega, min(max_omega, heading_gain * heading_error))
        forward = speed * max(0.0, math.cos(heading_error))
        yaw = _normalize_angle(yaw + omega * dt)
        x += forward * math.cos(yaw) * dt
        y += forward * math.sin(yaw) * dt
        t += dt
        track.append(TrackState(time=t, x=x, y=y, yaw=yaw))

    return track
