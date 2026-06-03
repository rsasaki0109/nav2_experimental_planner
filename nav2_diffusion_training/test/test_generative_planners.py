# Copyright 2026 nav2_diffusion_planner contributors
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
Train each generative planner family and verify the ONNX seam contract.

Each of flow / diffusion / consistency trains briefly, exports to ONNX, and is
loaded with onnxruntime to confirm it yields the [1, K, H, 3] tensor that the C++
OnnxTrajectoryModel consumes. Skipped when torch / onnxruntime are absent (CI).
"""

import os

import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('onnx')


@pytest.mark.parametrize('kind', ['flow', 'diffusion', 'consistency'])
def test_train_export_load_contract(kind, tmp_path):
    """A trained planner exports an ONNX model matching context[1,4]->[1,K,H,3]."""
    from nav2_diffusion_training.generative_planners import train_and_export

    path = os.path.join(str(tmp_path), kind + '.onnx')
    train_and_export(kind, path, num_samples=8, epochs=3)
    assert os.path.exists(path)

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    out = session.run(None, {'context': np.zeros((1, 4), dtype=np.float32)})[0]

    assert out.shape == (1, 3, 10, 3)
    assert np.isfinite(out).all()


def test_flow_training_reduces_loss():
    """Flow-matching loss decreases over a short training run."""
    from nav2_diffusion_training.generative_planners import (
        build_planner, make_synthetic_dataset)
    torch.manual_seed(0)
    model = build_planner('flow')
    inputs, targets = make_synthetic_dataset(16)
    expert = targets[:, 0, :, :]
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    first = model.flow_loss(inputs, expert).item()
    for _ in range(40):
        opt.zero_grad()
        loss = model.flow_loss(inputs, expert)
        loss.backward()
        opt.step()
    assert loss.item() < first
