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


def test_costmap_path_planner_exports_two_input_contract(tmp_path):
    """The costmap-conditioned path planner exports a context+costmap ONNX model."""
    from nav2_diffusion_training.path_planners import (
        PATH_COSTMAP_SIZE, PATH_H, PATH_K, train_and_export_costmap_path)

    path = os.path.join(str(tmp_path), 'costmap_path.onnx')
    train_and_export_costmap_path(path, num_samples=12, epochs=3)

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    names = {i.name for i in session.get_inputs()}
    assert names == {'context', 'costmap'}
    ctx = np.array([[4.0, 0.0]], dtype=np.float32)
    cm = np.zeros((1, 1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE), dtype=np.float32)
    out = session.run(None, {'context': ctx, 'costmap': cm})[0]
    assert out.shape == (1, PATH_K, PATH_H, 2)
    assert np.isfinite(out).all()


def test_costmap_path_loss_decreases_and_reads_costmap():
    """Costmap-conditioned flow loss decreases and the encoder uses the patch.

    The trained model's learned avoidance *direction* is checked deterministically
    in the C++ backend gtest (OnnxPathModelTest.CostmapConditionedVeersAwayFrom
    Obstacle); here we keep a fast check that training reduces the loss and that
    swapping the costmap changes the output (i.e. the patch is actually read).
    """
    from nav2_diffusion_training.path_planners import (
        _aligned_patch, CostmapPathFlowPlanner, make_costmap_path_dataset)
    torch.manual_seed(0)
    model = CostmapPathFlowPlanner()
    context, costmap, target = make_costmap_path_dataset(24)
    opt = torch.optim.Adam(model.parameters(), lr=0.02)
    first = model.flow_loss(context, costmap, target).item()
    for _ in range(30):
        opt.zero_grad()
        loss = model.flow_loss(context, costmap, target)
        loss.backward()
        opt.step()
    assert loss.item() < first
    model.eval()
    ctx = torch.tensor([[4.0, 0.0]])
    with torch.no_grad():
        left = model(ctx, _aligned_patch(1.0).unsqueeze(0))
        right = model(ctx, _aligned_patch(-1.0).unsqueeze(0))
    # Different costmaps must yield different proposals (the patch is read).
    assert (left - right).abs().max().item() > 1e-3


def test_costmap_path_transformer_exports_contract(tmp_path):
    """The transformer Mode B planner exports the same context+costmap ONNX seam."""
    from nav2_diffusion_training.path_planners import (
        PATH_COSTMAP_SIZE, PATH_H, PATH_K, train_and_export_costmap_path)

    path = os.path.join(str(tmp_path), 'costmap_path_transformer.onnx')
    train_and_export_costmap_path(
        path, num_samples=12, epochs=3, kind='transformer', dataset='both')

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    names = {i.name for i in session.get_inputs()}
    assert names == {'context', 'costmap'}
    ctx = np.array([[5.0, 0.0]], dtype=np.float32)
    cm = np.zeros((1, 1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE), dtype=np.float32)
    out = session.run(None, {'context': ctx, 'costmap': cm})[0]
    assert out.shape == (1, PATH_K, PATH_H, 2)
    assert np.isfinite(out).all()


def test_costmap_path_recurrent_exports_contract(tmp_path):
    """The recurrent Mode B planner exports the same context+costmap ONNX seam."""
    from nav2_diffusion_training.path_planners import (
        PATH_COSTMAP_SIZE, PATH_H, PATH_K, train_and_export_costmap_path)

    path = os.path.join(str(tmp_path), 'costmap_path_recurrent.onnx')
    train_and_export_costmap_path(
        path, num_samples=12, epochs=3, kind='recurrent', dataset='side')

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    names = {i.name for i in session.get_inputs()}
    assert names == {'context', 'costmap'}
    ctx = np.array([[5.0, 0.0]], dtype=np.float32)
    cm = np.zeros((1, 1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE), dtype=np.float32)
    out = session.run(None, {'context': ctx, 'costmap': cm})[0]
    assert out.shape == (1, PATH_K, PATH_H, 2)
    assert np.isfinite(out).all()


def test_costmap_path_recurrent_reads_costmap_and_veers():
    """Recurrent loss decreases, candidates roll out forward, and the patch is read.

    The full routing direction is guarded deterministically in the C++ backend
    gtest (CuratedZooPathRecurrentVeersAwayFromObstacle); here we keep a fast check
    that training reduces the loss, that the GRU rollout advances in x, and that
    swapping the obstacle side flips the mean lateral offset of the proposals.
    """
    from nav2_diffusion_training.path_planners import (
        _aligned_patch, CostmapPathRecurrentPlanner, make_costmap_path_dataset)
    torch.manual_seed(0)
    model = CostmapPathRecurrentPlanner()
    context, costmap, target = make_costmap_path_dataset(24)
    opt = torch.optim.Adam(model.parameters(), lr=0.02)
    first = model.recon_loss(context, costmap, target).item()
    for _ in range(80):
        opt.zero_grad()
        loss = model.recon_loss(context, costmap, target)
        loss.backward()
        opt.step()
    assert loss.item() < first
    model.eval()
    ctx = torch.tensor([[4.0, 0.0]])
    with torch.no_grad():
        # Obstacle on +y -> expert (and proposals) bow to the open -y side.
        left = model(ctx, _aligned_patch(1.0).unsqueeze(0))[0]    # [K, H, 2]
        right = model(ctx, _aligned_patch(-1.0).unsqueeze(0))[0]
    # The rollout advances forward (x grows from first to last waypoint).
    assert left[:, -1, 0].mean().item() > left[:, 0, 0].mean().item()
    # Swapping the obstacle side flips the mean lateral offset of the proposals.
    assert left[:, :, 1].mean().item() < right[:, :, 1].mean().item()


def test_gap_dataset_shapes_and_routes_through_slot():
    """The off-centre-gap dataset has the seam shapes and a slot-routing expert.

    The full routing *behaviour* of the trained model is guarded in the C++ backend
    gtest (OnnxPathModelTest.CuratedZooTransformerRoutesThroughGap); here we check
    the dataset shapes and that the expert path detours toward the off-centre slot.
    """
    from nav2_diffusion_training.path_planners import (
        PATH_DIM, PATH_H, make_costmap_path_gap_dataset)
    ctx, patches, targets = make_costmap_path_gap_dataset(12)
    assert ctx.shape[1] == 2
    assert patches.shape[1:] == (1, 24, 24)
    assert targets.shape[1:] == (PATH_H, PATH_DIM)
    # The first sample's slot is on +y, so the expert bows to +y at mid-path.
    assert targets[0, PATH_H // 2, 1].item() > 0.5


def test_footprint_penalty_prefers_routing_through_the_slot():
    """The footprint-clearance term penalizes a wall-crossing path over a slot one.

    A straight path crosses the wall (occupied); a path that detours to the slot
    offset at the wall stays in free space. The differentiable penalty must rank
    the slot path strictly lower — the signal that makes it validator-aware.
    """
    from nav2_diffusion_training.path_planners import (
        PATH_H, PATCH_FWD, _footprint_penalty, _gap_patch)
    slot_y = -2.0
    patch = _gap_patch(slot_y, x_lo=1.6, x_hi=2.4, slot_hw=0.5).unsqueeze(0)  # [1,1,24,24]
    xs = torch.linspace(0.0, PATCH_FWD * 0.7, PATH_H)
    straight = torch.stack([xs, torch.zeros(PATH_H)], dim=-1).view(1, 1, PATH_H, 2)
    t = torch.linspace(0.0, 1.0, PATH_H)
    bow = torch.exp(-((t - 0.5) / 0.22) ** 2) * slot_y           # dips to the slot at mid
    slot = torch.stack([xs, bow], dim=-1).view(1, 1, PATH_H, 2)
    pen_straight = _footprint_penalty(straight, patch, blur_sigma=2.5)
    pen_slot = _footprint_penalty(slot, patch, blur_sigma=2.5)
    assert pen_slot.item() < pen_straight.item()


def test_footprint_training_lowers_clearance_vs_recon_only(tmp_path):
    """The footprint term yields proposals with less occupancy overlap than recon.

    The full benchmark gap-threading is guarded in the C++ planner_benchmark; here
    we keep a fast CPU check that, trained for the same budget on the gap dataset,
    a recon+footprint model has a strictly lower footprint-clearance penalty than a
    recon-only model — the validator-aware signal at work — and that the 2-input
    ONNX still exports with the contract shape.
    """
    from nav2_diffusion_training.path_planners import (
        CostmapPathTransformerPlanner, _footprint_penalty, _path_dataset,
        train_and_export_costmap_path)
    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    context, costmap, target = _path_dataset('gap', 16)

    def fan_recon(out, model):
        tgt_x = target[..., 0].unsqueeze(1).expand(-1, model.fan.numel(), -1)
        tgt_y = target[..., 1].unsqueeze(1) + model.fan.view(1, -1, 1)
        return ((out - torch.stack([tgt_x, tgt_y], dim=-1)) ** 2).mean()

    def train(footprint):
        torch.manual_seed(0)
        model = CostmapPathTransformerPlanner()
        opt = torch.optim.Adam(model.parameters(), lr=0.01)
        for _ in range(150):
            opt.zero_grad()
            out = model(context, costmap)
            loss = fan_recon(out, model)
            if footprint:
                loss = loss + 3.0 * _footprint_penalty(out, costmap, blur_sigma=2.5)
            loss.backward()
            opt.step()
        with torch.no_grad():
            return _footprint_penalty(model(context, costmap), costmap, blur_sigma=2.5).item()

    assert train(footprint=True) < train(footprint=False)

    out = str(tmp_path / 'fp.onnx')
    train_and_export_costmap_path(
        out, num_samples=16, epochs=2, kind='transformer', dataset='gap',
        footprint=3.0, blur_sigma=2.5)
    sess = ort.InferenceSession(out)
    paths = sess.run(['paths'], {
        'context': np.array([[4.0, 0.0]], dtype=np.float32),
        'costmap': np.zeros((1, 1, 24, 24), dtype=np.float32)})[0]
    assert paths.shape == (1, 5, 12, 2)


def test_footprint_loss_rejected_for_flow_kind(tmp_path):
    """The footprint term needs the fan set-prediction kinds; flow must error."""
    from nav2_diffusion_training.path_planners import train_and_export_costmap_path
    with pytest.raises(ValueError):
        train_and_export_costmap_path(
            str(tmp_path / 'x.onnx'), num_samples=8, epochs=1, kind='flow',
            dataset='gap', footprint=1.0)
