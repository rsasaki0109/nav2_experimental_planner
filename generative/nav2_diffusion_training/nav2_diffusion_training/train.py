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
Minimal PyTorch training + ONNX export for the TrajectoryModel contract.

Trains a tiny model on synthetic rule-based expert data and exports it to the
exact I/O contract the C++ OnnxTrajectoryModel consumes (context [1, 4] ->
trajectories [1, K, H, 3]); see nav2_diffusion_onnx/README.md. PyTorch is a heavy
optional dependency, so this module is imported lazily (not from __init__) and
its test is skipped when torch is unavailable.
"""

from typing import List, Tuple

from nav2_diffusion_training.dataset import TrackState
from nav2_diffusion_training.experts import unicycle_to_goal
import torch
from torch import nn

NUM_CANDIDATES = 3
HORIZON = 10


class TinyPlanner(nn.Module):
    """Linear context -> [N, K, H, 3] trajectory tensor (a training skeleton)."""

    def __init__(self):
        super().__init__()
        self.linear = nn.Linear(4, NUM_CANDIDATES * HORIZON * 3)

    def forward(self, context):
        """Map a [N, 4] context batch to a [N, K, H, 3] trajectory tensor."""
        return self.linear(context).reshape(-1, NUM_CANDIDATES, HORIZON, 3)


def _resample(track: List[TrackState], steps: int) -> List[List[float]]:
    """Resample a variable-length track to exactly `steps` (x, y, yaw) rows."""
    if not track:
        return [[0.0, 0.0, 0.0] for _ in range(steps)]
    rows = []
    last = len(track) - 1
    for k in range(steps):
        idx = round(k * last / max(1, steps - 1))
        state = track[idx]
        rows.append([state.x, state.y, state.yaw])
    return rows


def make_synthetic_dataset(
    num_samples: int, speed: float = 0.3, max_angular: float = 1.0,
) -> Tuple['torch.Tensor', 'torch.Tensor']:
    """Build (context, target) tensors from rule-based expert rollouts."""
    contexts = []
    targets = []
    for i in range(num_samples):
        goal_x = 1.0 + 0.3 * (i % 5)
        goal_y = -1.0 + 0.4 * (i % 6)
        track = unicycle_to_goal(
            TrackState(time=0.0, x=0.0, y=0.0, yaw=0.0),
            goal_x, goal_y, speed=speed, max_omega=max_angular)
        rows = _resample(track, HORIZON)
        contexts.append([goal_x, goal_y, speed, max_angular])
        targets.append([rows for _ in range(NUM_CANDIDATES)])
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.tensor(targets, dtype=torch.float32),
    )


def export_onnx(model: nn.Module, path: str) -> None:
    """Export a planner to ONNX with the C++ backend's I/O names."""
    model.eval()
    dummy = torch.zeros(1, 4)
    torch.onnx.export(
        model, dummy, path,
        input_names=['context'], output_names=['trajectories'],
        opset_version=18)


def train_and_export(
    path: str, num_samples: int = 16, epochs: int = 5, lr: float = 0.01,
) -> float:
    """Train TinyPlanner on synthetic expert data, export ONNX, return final loss."""
    torch.manual_seed(0)
    model = TinyPlanner()
    inputs, targets = make_synthetic_dataset(num_samples)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        loss = loss_fn(model(inputs), targets)
        loss.backward()
        optimizer.step()

    export_onnx(model, path)
    return float(loss.item())
