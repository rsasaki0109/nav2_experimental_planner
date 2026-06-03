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
Generative GLOBAL-path model for the Nav2 Mode B PathModel ONNX seam.

No surveyed work open-sources a generative model integrated as a Nav2
nav2_core::GlobalPlanner; this is that model behind the PathModel seam. A
flow-matching network proposes K candidate start->goal paths in a goal-aligned
frame (goal placed at (d, 0)), so it only has to learn the distribution of
smooth paths to a goal at distance d, and the C++ backend rotates/translates the
output back into the map frame. It exports to the contract

    context [1, 2] = [goal_distance, 0]  ->  paths [1, K, H, 2]   (x, y aligned)

mirroring the trajectory model's [1, 4] -> [1, K, H, 3]. The deterministic
costmap-validation layer in the planner decides which proposal (if any) is used.

PyTorch is a heavy optional dependency, imported here (not from __init__).
"""

import onnx
import torch
from torch import nn

PATH_K = 5            # candidate paths proposed
PATH_H = 12           # waypoints per path
PATH_DIM = 2          # x, y (aligned frame)
PATH_VEC = PATH_H * PATH_DIM
CTX_DIM = 2           # [goal_distance, 0]


class _PathMLP(nn.Module):
    """Small MLP mapping (path, context, time) -> path-sized velocity."""

    def __init__(self, hidden=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(PATH_VEC + CTX_DIM + 1, hidden), nn.SiLU(),
            nn.Linear(hidden, hidden), nn.SiLU(),
            nn.Linear(hidden, PATH_VEC),
        )

    def forward(self, path, context, t):
        return self.net(torch.cat([path, context, t], dim=-1))


def _fixed_latents(seed):
    """K deterministic latent seeds baked into the exported graph as a buffer."""
    gen = torch.Generator().manual_seed(seed)
    return torch.randn(PATH_K, PATH_VEC, generator=gen)


class PathFlowPlanner(nn.Module):
    """Conditional flow-matching generative global-path planner."""

    def __init__(self, steps=4):
        super().__init__()
        self.field = _PathMLP()
        self.steps = steps
        self.register_buffer('latents', _fixed_latents(11))

    def velocity(self, x, context, t):
        return self.field(x, context, t)

    def flow_loss(self, context, target):
        """Conditional flow-matching loss: regress the straight-line velocity."""
        b = target.shape[0]
        x1 = target.reshape(b, PATH_VEC)
        x0 = torch.randn_like(x1)
        t = torch.rand(b, 1)
        xt = (1.0 - t) * x0 + t * x1
        pred = self.velocity(xt, context, t)
        return ((pred - (x1 - x0)) ** 2).mean()

    def _integrate(self, x0, context):
        x = x0
        dt = 1.0 / self.steps
        for i in range(self.steps):
            t = torch.full((x.shape[0], 1), i * dt)
            x = x + self.velocity(x, context, t) * dt
        return x

    def forward(self, context):
        outs = []
        for k in range(PATH_K):
            z = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            outs.append(self._integrate(z, context).reshape(-1, 1, PATH_H, PATH_DIM))
        return torch.cat(outs, dim=1)


def make_path_dataset(num_samples):
    """
    Synthetic smooth start->goal paths in the goal-aligned frame.

    The goal sits at (d, 0); each expert path is the straight line plus a
    half-sine lateral bow of random signed amplitude, so the proposal
    distribution is multimodal (left/right detours of varying strength) and the
    K fixed latents spread to cover it.
    """
    contexts = []
    targets = []
    for i in range(num_samples):
        # Deterministic sweep over distance and bow amplitude (no RNG so the
        # exported fixture is reproducible across runs).
        d = 1.0 + 5.0 * ((i * 7) % 11) / 10.0
        amp = (-0.4 + 0.8 * ((i * 3) % 9) / 8.0) * d
        rows = []
        for h in range(PATH_H):
            t = h / (PATH_H - 1)
            rows.append([t * d, amp * (t * (1.0 - t) * 4.0)])  # 0 at both ends
        contexts.append([d, 0.0])
        targets.append(rows)
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.tensor(targets, dtype=torch.float32),
    )


def train_and_export_path(path, num_samples=88, epochs=300, lr=0.01, steps=4):
    """Train PathFlowPlanner and export it to the PathModel ONNX seam contract."""
    torch.manual_seed(0)
    model = PathFlowPlanner(steps=steps)
    context, target = make_path_dataset(num_samples)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        loss = model.flow_loss(context, target)
        out = model(context[:16])                          # [B, K, H, 2]
        jerk = out[:, :, 2:, :] - 2 * out[:, :, 1:-1, :] + out[:, :, :-2, :]
        loss = loss + 2.0 * (jerk ** 2).mean()             # smoothness
        loss.backward()
        optimizer.step()

    model.eval()
    dummy = torch.zeros(1, CTX_DIM)
    torch.onnx.export(
        model, dummy, path,
        input_names=['context'], output_names=['paths'],
        opset_version=18)
    onnx.save_model(onnx.load(path), path, save_as_external_data=False)
    return float(loss.item())
