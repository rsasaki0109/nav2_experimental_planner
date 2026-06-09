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

"""Tests for the torch-free comparison metrics in model_eval."""

from nav2_diffusion_training.model_eval import (
    CLEAR_CAP, collides, endpoint_progress, evaluate_candidates,
    is_finite_traj, min_clearance, patch_obstacle_xy, rank_rows, select_best,
    total_turning)


def _straight(length=0.3, n=10, lat=0.0):
    """Build a forward trajectory at a fixed lateral offset."""
    return [(length * (h + 1) / n, lat, 0.0) for h in range(n)]


def test_patch_obstacle_xy_maps_center_cell_to_origin():
    # 3x3 patch, only the center cell occupied -> base-frame (0, 0).
    patch = [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    xy = patch_obstacle_xy(patch, res=0.1)
    assert xy == [(0.0, 0.0)]


def test_min_clearance_and_collision():
    obstacles = [(0.2, 0.0)]
    near = _straight(lat=0.0)            # passes through x=0.2, y=0
    far = _straight(lat=0.5)             # 0.5 m to the side
    assert min_clearance(near, obstacles) < 0.05
    assert collides(near, obstacles, radius=0.06)
    assert not collides(far, obstacles, radius=0.06)
    # No obstacles -> capped clearance, never collides.
    assert min_clearance(near, []) == CLEAR_CAP


def test_diverged_trajectory_scored_as_failure():
    bad = [(1e6, -1e6, 0.0)] + _straight()[1:]
    assert not is_finite_traj(bad)
    assert collides(bad, [(0.2, 0.0)], radius=0.06)   # invalid -> counts as hit
    assert min_clearance(bad, [(0.2, 0.0)]) == 0.0
    assert endpoint_progress(bad, 0.3, 0.0) >= 0.0    # finite, no exception


def test_endpoint_progress_bounds():
    goal = (0.3, 0.0)
    assert endpoint_progress(_straight(length=0.3), *goal) > 0.9   # reaches goal
    assert endpoint_progress(_straight(length=0.0), *goal) == 0.0  # no movement


def test_total_turning_zero_for_straight():
    assert total_turning(_straight()) == 0.0
    turning = [(0.1 * i, 0.0, 0.2 * i) for i in range(5)]
    assert total_turning(turning) > 0.0


def test_select_best_prefers_safe_high_progress():
    obstacles = [(0.2, 0.0)]
    blocked = _straight(lat=0.0)         # most progress but collides
    safe = _straight(lat=0.3)            # safe, slightly less direct
    best = select_best([blocked, safe], 0.3, 0.0, obstacles, radius=0.06)
    assert best is safe


def test_evaluate_candidates_reports_success():
    obstacles = [(0.2, 0.0)]
    cands = [_straight(lat=0.0), _straight(lat=0.3)]
    m = evaluate_candidates(cands, 0.3, 0.0, obstacles, radius=0.06)
    assert m['success'] == 1.0           # one safe candidate exists
    assert m['collision_rate'] == 0.5    # one of two collides

    all_blocked = [_straight(lat=0.0), _straight(lat=0.01)]
    m2 = evaluate_candidates(all_blocked, 0.3, 0.0, obstacles, radius=0.06)
    assert m2['success'] == 0.0


def test_rank_rows_orders_by_safety_first():
    rows = [
        {'name': 'safe', 'family': 'f', 'conditioning': 'c', 'steps': 1,
         'success': 1.0, 'clearance': 0.3, 'progress': 0.6, 'turning': 0.0,
         'collision_rate': 0.0},
        {'name': 'unsafe', 'family': 'f', 'conditioning': 'c', 'steps': 1,
         'success': 0.0, 'clearance': 0.1, 'progress': 0.9, 'turning': 0.2,
         'collision_rate': 1.0},
    ]
    ranked = rank_rows(rows)
    assert ranked[0]['name'] == 'safe'   # success dominates raw progress
    assert ranked[0]['score'] > ranked[1]['score']
