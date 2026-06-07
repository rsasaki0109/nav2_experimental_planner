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
    """The corrected reactive dodge oracle clears obstacles in closed loop (4/4 of old).

    The shipped oracle collided on every obstacle scenario (a 0.20 m transient bow + an
    on-line carrot); the corrected one (sustained free-side offset, curvature-based
    commit) drives expert-only to the goal without collision on the dead-ahead frontal
    block and the off-centre side block. Guards the data DAgger aggregates
    (docs/generative_limits.md).
    """
    from nav2_diffusion_training.dagger import SCENARIOS, rollout

    by_name = {sc[0]: sc for sc in SCENARIOS}
    for name in ('frontal', 'side', 'two'):
        reached, collided, _ = rollout(None, by_name[name], collect=False, expert_only=True)
        assert reached, f'corrected oracle failed to reach goal on {name}'
        assert not collided, f'corrected oracle collided on {name}'


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
