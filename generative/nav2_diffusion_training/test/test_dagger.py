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
Smoke-test the DAgger closed-loop training loop for the Mode A trajectory model.

Verifies the loop runs end to end (rollout in the costmap sim, expert relabel,
aggregate, retrain) and exports an ONNX matching context[1,4]->[1,3,10,3], the
contract the C++ OnnxTrajectoryModel consumes. Skipped when torch / onnxruntime
are absent (CI). The model quality itself is a separate, documented concern
(docs/generative_limits.md): DAgger is the right tool for the distribution shift,
but the small synthetic model only gains marginally — this test guards the loop,
not the model's competence.
"""

import pytest

pytest.importorskip('torch')
pytest.importorskip('onnx')


def test_dagger_runs_and_exports_contract(tmp_path):
    """A tiny DAgger run aggregates visited states and exports the ONNX contract."""
    import numpy as np
    import onnxruntime as ort

    from nav2_diffusion_training.dagger import dagger_train_costmap

    path = str(tmp_path / 'dagger.onnx')
    dagger_train_costmap(path, iters=1, base_samples=16, epochs=2, lr=0.01)

    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    names = {i.name for i in session.get_inputs()}
    assert names == {'context', 'costmap'}
    out = session.run(
        None,
        {'context': np.zeros((1, 4), dtype=np.float32),
         'costmap': np.zeros((1, 1, 32, 32), dtype=np.float32)})[0]
    assert out.shape == (1, 3, 10, 3)
    assert np.isfinite(out).all()


def test_dagger_collects_visited_states():
    """Rolling the policy out yields (context, patch, expert-label) samples."""
    import numpy as np

    from nav2_diffusion_training.dagger import (
        CostmapFlowPlanner, SCENARIOS, rollout)

    model = CostmapFlowPlanner(steps=4).eval()
    _, _, samples = rollout(model, SCENARIOS[1], collect=True)  # 'frontal' scenario
    assert len(samples) > 0
    ctx, patch, target = samples[0]
    assert len(ctx) == 4
    assert np.asarray(patch).shape == (32, 32)
    assert np.asarray(target).shape == (10, 3)


def test_corrected_oracle_reaches_goal_collision_free():
    """The corrected reactive dodge oracle clears every obstacle scenario in closed loop.

    The shipped oracle collided / stalled (a 0.20 m transient bow + an on-line carrot, and
    a dead-ahead block it could not thread); the corrected one threads each scenario
    expert-only — a *sustained* committed offset to the free side (held until the block is
    passed, so the carrot cannot snap it back), and a corridor left to the carrot to centre.
    If the oracle itself cannot thread a scenario, no model trained on it can, so this
    guards the data DAgger aggregates (docs/generative_limits.md).
    """
    from nav2_diffusion_training.dagger import SCENARIOS, rollout

    for sc in SCENARIOS:
        reached, collided, _ = rollout(None, sc, collect=False, expert_only=True)
        assert reached, f'corrected oracle failed to reach goal on {sc[0]}'
        assert not collided, f'corrected oracle collided on {sc[0]}'


def test_dodge_commits_a_side_for_a_block_and_steers_off_a_wall():
    """``_dodge_offset`` commits to one free side for a block and steers off a near wall.

    A dead-ahead block must be dodged decisively to ONE side (a held offset, so the robot
    threads it instead of stalling); a wall close on one side must be steered away from
    (toward the open centre); a clear patch must not be dodged at all
    (docs/generative_limits.md).
    """
    from nav2_diffusion_training.dagger import (
        build_costmap, crop_patch, _dodge_offset)

    # Dead-ahead centred block 0.5 m ahead -> a non-zero committed offset to one side.
    block = crop_patch(build_costmap([(3.0, 3.0, 4)]), 2.5, 3.0, 0.0)
    assert abs(_dodge_offset(block)) > 0.1, 'a frontal block must be dodged'

    # Near the top wall of a corridor -> steer toward -y (away from the wall, to the open
    # centre).
    near_wall = crop_patch(build_costmap([('wall', 3.9, 2)]), 1.0, 3.6, 0.0)
    assert _dodge_offset(near_wall) < 0.0, 'must steer away from a close wall'

    # Clear patch -> no dodge.
    import numpy as np
    assert _dodge_offset(np.zeros((32, 32), dtype=np.float32)) == 0.0


def test_corridor_scenario_present_for_centring():
    """The DAgger scenarios include the two-walled corridor the benchmark centres on."""
    from nav2_diffusion_training.dagger import SCENARIOS, build_costmap

    by_name = {sc[0]: sc for sc in SCENARIOS}
    assert 'corridor' in by_name, 'corridor centring scenario must be trained on'
    # Its costmap has both walls occupied (full-width horizontal bands).
    gm = build_costmap(by_name['corridor'][1])
    assert gm.sum() > 0
    assert SCENARIOS[1][0] == 'frontal'  # index contract for test_dagger_collects...


def test_dagger_transformer_exports_contract(tmp_path):
    """A tiny transformer-DAgger run exports the Mode A ONNX contract."""
    import numpy as np
    import onnxruntime as ort

    from nav2_diffusion_training.dagger import dagger_train_costmap_transformer

    path = str(tmp_path / 'threading.onnx')
    dagger_train_costmap_transformer(path, iters=1, base_samples=16, epochs=2, lr=0.003)

    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    assert {i.name for i in session.get_inputs()} == {'context', 'costmap'}
    out = session.run(
        None,
        {'context': np.zeros((1, 4), dtype=np.float32),
         'costmap': np.zeros((1, 1, 32, 32), dtype=np.float32)})[0]
    assert out.shape == (1, 3, 10, 3)
    assert np.isfinite(out).all()
