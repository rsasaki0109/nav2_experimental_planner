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
Offline, deterministic comparison metrics for the generative planner families.

The live ``benchmark_runner`` node records executed odometry from a running
Nav2/sim stack; that path needs a simulator. This module instead scores the
*proposals* a generative model makes on identical synthetic obstacle scenarios,
so the flow / diffusion / consistency families (context-only and costmap-
conditioned) can be ranked on CPU with no ROS comms.

Trajectories are base-frame point lists ``[(x, y, yaw), ...]`` (metres, rad,
x forward / y left). Obstacles are given as base-frame ``(x, y)`` cell centres.
The pure-geometry helpers here carry no torch dependency so they unit-test
without the heavy optional model stack.
"""

import math

# Egocentric patch resolution [m/cell]; matches the costmap demo / training grid.
PATCH_RES = 0.03
# A local trajectory point this far from the robot is non-physical: the model
# diverged (e.g. DDIM dividing by ~0 alpha). Such candidates are scored as
# failures, not as "infinitely clear".
COORD_BOUND = 5.0
# Clearance beyond this (patch is ~1 m across) carries no extra safety value.
CLEAR_CAP = 1.5
# Worst-case turning assigned to a diverged trajectory [rad].
TURN_CAP = 2.0 * math.pi * 4.0


def is_finite_traj(traj, bound=COORD_BOUND):
    """Return False if any coordinate is non-finite or beyond the bound."""
    for x, y, yaw in traj:
        for v in (x, y, yaw):
            if not math.isfinite(v):
                return False
        if abs(x) > bound or abs(y) > bound:
            return False
    return True


def patch_obstacle_xy(patch, res=PATCH_RES):
    """
    Convert occupied cells of an egocentric patch to base-frame (x, y) metres.

    ``patch`` is a 2D row-major grid (row 0 = furthest ahead) with values in
    [0, 1]; cells > 0.5 are treated as obstacles. Mapping mirrors the costmap
    demo: x = (center - row) * res (forward), y = (center - col) * res (left).
    """
    rows = len(patch)
    cols = len(patch[0]) if rows else 0
    center = (rows - 1) / 2.0
    ccol = (cols - 1) / 2.0
    out = []
    for r in range(rows):
        for c in range(cols):
            if patch[r][c] > 0.5:
                out.append(((center - r) * res, (ccol - c) * res))
    return out


def min_clearance(traj, obstacles):
    """
    Return the minimum clearance [m] from the trajectory to any obstacle.

    The distance is capped at ``CLEAR_CAP``; a diverged (non-finite) trajectory
    has clearance 0.
    """
    if not is_finite_traj(traj):
        return 0.0
    if not obstacles:
        return CLEAR_CAP
    best = CLEAR_CAP
    for x, y, _ in traj:
        for ox, oy in obstacles:
            d = math.hypot(x - ox, y - oy)
            if d < best:
                best = d
    return best


def collides(traj, obstacles, radius):
    """Return True if the trajectory diverged or is within ``radius`` [m]."""
    if not is_finite_traj(traj):
        return True
    return min_clearance(traj, obstacles) < radius


def endpoint_progress(traj, goal_x, goal_y):
    """
    Fraction of the goal distance closed by the trajectory endpoint, in [0, 1].

    0 when the endpoint is no closer than the start (origin); 1 at the goal.
    """
    if not is_finite_traj(traj):
        return 0.0
    start = math.hypot(goal_x, goal_y)
    if start <= 1e-6:
        return 1.0
    ex, ey, _ = traj[-1]
    end = math.hypot(goal_x - ex, goal_y - ey)
    return max(0.0, min(1.0, 1.0 - end / start))


def total_turning(traj):
    """Sum of absolute yaw changes [rad] along the trajectory (lower is smoother)."""
    if not is_finite_traj(traj):
        return TURN_CAP
    return min(TURN_CAP, sum(abs(traj[i][2] - traj[i - 1][2]) for i in range(1, len(traj))))


def select_best(candidates, goal_x, goal_y, obstacles, radius):
    """
    Pick the candidate a safety-first controller would choose.

    Returns the highest-progress collision-free candidate, falling back to the
    highest-clearance candidate when every candidate collides (mirrors the
    controller's fallback intent).
    """
    free = [c for c in candidates if not collides(c, obstacles, radius)]
    if free:
        return max(free, key=lambda c: endpoint_progress(c, goal_x, goal_y))
    return max(candidates, key=lambda c: min_clearance(c, obstacles))


def evaluate_candidates(candidates, goal_x, goal_y, obstacles, radius):
    """
    Score one model's candidate set on a single scenario.

    Returns a dict with the selected candidate's clearance / progress / turning,
    plus the collision rate across all candidates.
    """
    free = [c for c in candidates if not collides(c, obstacles, radius)]
    best = select_best(candidates, goal_x, goal_y, obstacles, radius)
    n_collide = len(candidates) - len(free)
    return {
        # success: did the model propose at least one safe candidate the
        # controller could pick? This is the operative safety outcome in the
        # propose/dispose/select architecture.
        'success': 1.0 if free else 0.0,
        'clearance': min_clearance(best, obstacles),
        'progress': endpoint_progress(best, goal_x, goal_y),
        'turning': total_turning(best),
        'collision_rate': n_collide / max(1, len(candidates)),
    }


def _minmax_norm(values, higher_is_better):
    """Min-max normalise to [0, 1]; flat columns map to 1.0 (no discrimination)."""
    lo, hi = min(values), max(values)
    if hi - lo < 1e-9:
        return [1.0 for _ in values]
    norm = [(v - lo) / (hi - lo) for v in values]
    return norm if higher_is_better else [1.0 - n for n in norm]


# Safety-first weighting, matching nav2_diffusion_benchmarks ScoreWeights
# (safety 0.5 / progress 0.3 / smoothness 0.2). Within safety, clearance
# dominates collision-freedom 0.7 / 0.3.
W_SAFETY, W_PROGRESS, W_SMOOTH = 0.5, 0.3, 0.2


def rank_rows(rows):
    """
    Turn per-model aggregate metrics into scored, ranked leaderboard rows.

    ``rows`` is a list of dicts each with keys: name, family, conditioning,
    steps, success, clearance, progress, turning, collision_rate. Adds ``score``
    (higher is better) and returns the list sorted by score descending.

    Safety combines the absolute success rate (fraction of scenarios with at
    least one safe candidate) with the normalised clearance margin; progress and
    smoothness are min-max normalised across models.
    """
    clr = _minmax_norm([r['clearance'] for r in rows], higher_is_better=True)
    prog = _minmax_norm([r['progress'] for r in rows], higher_is_better=True)
    turn = _minmax_norm([r['turning'] for r in rows], higher_is_better=False)
    for i, r in enumerate(rows):
        safety = 0.7 * r['success'] + 0.3 * clr[i]
        r['score'] = W_SAFETY * safety + W_PROGRESS * prog[i] + W_SMOOTH * turn[i]
    return sorted(rows, key=lambda r: r['score'], reverse=True)
