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
Train the generative global-path planner and verify the PathModel ONNX contract.

The model trains briefly, exports to ONNX, and is loaded with onnxruntime to
confirm it yields the [1, K, H, 2] tensor that the C++ OnnxPathModel consumes.
Skipped when torch / onnxruntime are absent (CI).
"""

import os

import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('onnx')


def test_path_planner_exports_contract(tmp_path):
    """A trained path planner exports an ONNX model matching context[1,2]->[1,K,H,2]."""
    from nav2_diffusion_training.path_planners import (
        PATH_H, PATH_K, train_and_export_path)

    path = os.path.join(str(tmp_path), 'path_flow.onnx')
    train_and_export_path(path, num_samples=16, epochs=3)
    assert os.path.exists(path)

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    names = {i.name for i in session.get_inputs()}
    assert names == {'context'}
    out = session.run(None, {'context': np.array([[3.0, 0.0]], dtype=np.float32)})[0]
    assert out.shape == (1, PATH_K, PATH_H, 2)
    assert np.isfinite(out).all()


def test_path_endpoints_track_goal_distance():
    """Candidate endpoints scale with the goal distance and start near origin."""
    from nav2_diffusion_training.path_planners import (
        PATH_H, make_path_dataset, PathFlowPlanner)
    torch.manual_seed(0)
    model = PathFlowPlanner()
    context, target = make_path_dataset(64)
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    for _ in range(120):
        opt.zero_grad()
        loss = model.flow_loss(context, target)
        loss.backward()
        opt.step()
    model.eval()
    with torch.no_grad():
        near = model(torch.tensor([[2.0, 0.0]]))[0]   # [K, H, 2]
        far = model(torch.tensor([[5.0, 0.0]]))[0]
    # Endpoints advance further for the more distant goal.
    assert far[:, -1, 0].mean().item() > near[:, -1, 0].mean().item()
    # Paths start near the origin (within a loose regression tolerance).
    assert abs(near[:, 0, 0].mean().item()) < 0.5
    assert PATH_H == near.shape[1]


def test_path_flow_training_reduces_loss():
    """Flow-matching loss decreases over a short training run."""
    from nav2_diffusion_training.path_planners import (
        make_path_dataset, PathFlowPlanner)
    torch.manual_seed(0)
    model = PathFlowPlanner()
    context, target = make_path_dataset(32)
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    first = model.flow_loss(context, target).item()
    for _ in range(60):
        opt.zero_grad()
        loss = model.flow_loss(context, target)
        loss.backward()
        opt.step()
    assert loss.item() < first
