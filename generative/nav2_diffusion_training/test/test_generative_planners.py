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
Train each generative planner family and verify the ONNX seam contract.

Each of flow / diffusion / consistency trains briefly, exports to ONNX, and is
loaded with onnxruntime to confirm it yields the [1, K, H, 3] tensor that the C++
OnnxTrajectoryModel consumes. Skipped when torch / onnxruntime are absent (CI).
"""

import os

import pytest

torch = pytest.importorskip('torch')
pytest.importorskip('onnx')


@pytest.mark.parametrize('kind', ['flow', 'diffusion', 'consistency', 'transformer', 'recurrent'])
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


@pytest.mark.parametrize('kind', ['flow', 'diffusion', 'consistency', 'transformer', 'recurrent'])
def test_costmap_conditioned_exports_two_input_onnx(kind, tmp_path):
    """Each costmap-conditioned family exports a context+costmap ONNX model."""
    from nav2_diffusion_training.generative_planners import (
        train_and_export_costmap, COSTMAP_SIZE)

    path = os.path.join(str(tmp_path), 'costmap_' + kind + '.onnx')
    train_and_export_costmap(path, kind=kind, num_samples=8, epochs=3)

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    names = {i.name for i in session.get_inputs()}
    assert names == {'context', 'costmap'}
    ctx = np.zeros((1, 4), dtype=np.float32)
    cm = np.zeros((1, 1, COSTMAP_SIZE, COSTMAP_SIZE), dtype=np.float32)
    out = session.run(None, {'context': ctx, 'costmap': cm})[0]
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


def test_costmap_transformer_reads_costmap_and_reduces_loss():
    """
    The transformer set-prediction planner fits the costmap-aware expert.

    A short run reduces the reconstruction loss, and after training every candidate
    veers away from a one-sided obstacle — i.e. the queries actually attend to the
    costmap rather than emitting a fixed shape.
    """
    from nav2_diffusion_training.generative_planners import (
        CostmapTransformerPlanner, make_costmap_dataset)
    torch.manual_seed(0)
    model = CostmapTransformerPlanner()
    context, costmap, target = make_costmap_dataset(48)
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    first = model.recon_loss(context, costmap, target).item()
    for _ in range(200):
        opt.zero_grad()
        loss = model.recon_loss(context, costmap, target)
        loss.backward()
        opt.step()
    assert loss.item() < first

    # Obstacle on +y (left, low columns) -> every candidate should bow to -y. The
    # expert's half-sine bow is zero at both endpoints, so test the mean lateral
    # offset over the horizon rather than the final point.
    model.eval()
    s = costmap.shape[-1]
    patch = torch.zeros(1, 1, s, s)
    patch[:, :, s // 4:s // 2, 0:s // 2] = 1.0
    ctx = torch.tensor([[1.0, 0.0, 0.3, 1.0]], dtype=torch.float32)
    with torch.no_grad():
        out = model(ctx, patch)               # [1, K, H, 3]
    assert (out[0, :, :, 1].mean(dim=1) < 0.0).all()   # mean lateral offset is -y


def test_costmap_recurrent_reads_costmap_and_rolls_out():
    """
    The recurrent (GRU) rollout planner fits the costmap-aware expert.

    A short run reduces the reconstruction loss, the autoregressive rollout drives
    forward (x increases along the horizon), and every candidate bows away from a
    one-sided obstacle — i.e. the rollout conditioning reads the costmap rather than
    emitting a fixed shape.
    """
    from nav2_diffusion_training.generative_planners import (
        CostmapRecurrentPlanner, make_costmap_dataset)
    torch.manual_seed(0)
    model = CostmapRecurrentPlanner()
    # A compact dataset / short run keeps this test light: the autoregressive rollout
    # unrolls into a deep graph (K x HORIZON GRU cells), so each epoch is far heavier
    # than the one-shot families. By ~60 epochs the veer is fully resolved; 100 with
    # lr 0.02 leaves margin against seed / hardware variation.
    context, costmap, target = make_costmap_dataset(16)
    opt = torch.optim.Adam(model.parameters(), lr=0.02)
    first = model.recon_loss(context, costmap, target).item()
    for _ in range(100):
        opt.zero_grad()
        loss = model.recon_loss(context, costmap, target)
        loss.backward()
        opt.step()
    assert loss.item() < first

    model.eval()
    s = costmap.shape[-1]
    patch = torch.zeros(1, 1, s, s)
    patch[:, :, s // 4:s // 2, 0:s // 2] = 1.0   # obstacle on +y -> bow to -y
    ctx = torch.tensor([[1.0, 0.0, 0.3, 1.0]], dtype=torch.float32)
    with torch.no_grad():
        out = model(ctx, patch)               # [1, K, H, 3]
    assert (out[0, :, :, 1].mean(dim=1) < 0.0).all()   # mean lateral offset is -y
    assert (out[0, :, -1, 0] > out[0, :, 0, 0]).all()   # rolls forward over horizon
