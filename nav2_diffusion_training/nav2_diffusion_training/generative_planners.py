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
Generative trajectory-model families for the TrajectoryModel ONNX seam.

Implements three goal-conditioned generative local planners that no surveyed
work open-sources for ROS 2 Nav2 ground robots: flow matching (single/few-step),
diffusion (DDIM), and a one-step distilled (consistency-style) planner. Each maps
a context vector [goal_x, goal_y, linear_speed, max_angular] to K candidate SE(2)
trajectories and exports to the C++ OnnxTrajectoryModel contract
context [1, 4] -> trajectories [1, K, H, 3].

PyTorch is a heavy optional dependency, imported here (not from __init__); tests
skip when torch is unavailable.
"""

from nav2_diffusion_training.train import make_synthetic_dataset
import onnx
import torch
from torch import nn

NUM_CANDIDATES = 3
HORIZON = 10
DIM = 3                       # x, y, yaw
TRAJ = HORIZON * DIM
DIFFUSION_STEPS = 16


class _MLP(nn.Module):
    """Small MLP mapping (trajectory, context, time) -> trajectory-sized output."""

    def __init__(self, hidden=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(TRAJ + 4 + 1, hidden), nn.SiLU(),
            nn.Linear(hidden, hidden), nn.SiLU(),
            nn.Linear(hidden, TRAJ),
        )

    def forward(self, traj, context, t):
        return self.net(torch.cat([traj, context, t], dim=-1))


def _fixed_latents(seed=0):
    """K deterministic latent seeds, baked into the exported graph as a buffer."""
    gen = torch.Generator().manual_seed(seed)
    return torch.randn(NUM_CANDIDATES, TRAJ, generator=gen)


class FlowMatchingPlanner(nn.Module):
    """Single/few-step conditional flow matching (GoalFlow-style, design 5.3)."""

    def __init__(self, steps=1):
        super().__init__()
        self.field = _MLP()
        self.steps = steps
        self.register_buffer('latents', _fixed_latents(1))

    def velocity(self, x, context, t):
        return self.field(x, context, t)

    def flow_loss(self, context, target):
        """Conditional flow-matching loss: regress the straight-line velocity."""
        b = target.shape[0]
        x1 = target.reshape(b, TRAJ)
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
        for k in range(NUM_CANDIDATES):
            z = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            outs.append(self._integrate(z, context).reshape(-1, 1, HORIZON, DIM))
        return torch.cat(outs, dim=1)


def _alpha_bar(steps):
    """Cosine-schedule cumulative alphas for the diffusion process."""
    t = torch.linspace(0, 1, steps + 1)
    f = torch.cos((t + 0.008) / 1.008 * torch.pi / 2) ** 2
    return (f / f[0])[1:]


class DiffusionPlanner(nn.Module):
    """DDPM-trained, DDIM-sampled diffusion planner (JLAP-style families)."""

    def __init__(self, sample_steps=4):
        super().__init__()
        self.eps = _MLP()
        self.sample_steps = sample_steps
        self.register_buffer('alpha_bar', _alpha_bar(DIFFUSION_STEPS))
        self.register_buffer('latents', _fixed_latents(2))

    def diffusion_loss(self, context, target):
        """Predict the noise added to the target at a random timestep."""
        b = target.shape[0]
        x0 = target.reshape(b, TRAJ)
        ti = torch.randint(0, DIFFUSION_STEPS, (b,))
        ab = self.alpha_bar[ti].unsqueeze(-1)
        noise = torch.randn_like(x0)
        xt = ab.sqrt() * x0 + (1 - ab).sqrt() * noise
        pred = self.eps(xt, context, (ti.float() / DIFFUSION_STEPS).unsqueeze(-1))
        return ((pred - noise) ** 2).mean()

    def _ddim(self, x, context):
        idx = torch.linspace(DIFFUSION_STEPS - 1, 0, self.sample_steps).long()
        for j in range(self.sample_steps):
            i = int(idx[j])
            ab = self.alpha_bar[i]
            t = torch.full((x.shape[0], 1), float(i) / DIFFUSION_STEPS)
            eps = self.eps(x, context, t)
            x0 = (x - (1 - ab).sqrt() * eps) / ab.sqrt()
            if j + 1 < self.sample_steps:
                ab_next = self.alpha_bar[int(idx[j + 1])]
                x = ab_next.sqrt() * x0 + (1 - ab_next).sqrt() * eps
            else:
                x = x0
        return x

    def forward(self, context):
        outs = []
        for k in range(NUM_CANDIDATES):
            z = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            outs.append(self._ddim(z, context).reshape(-1, 1, HORIZON, DIM))
        return torch.cat(outs, dim=1)


class ConsistencyPlanner(nn.Module):
    """One-step distilled planner (consistency-style): map any noised state to x0."""

    def __init__(self):
        super().__init__()
        self.f = _MLP()
        self.register_buffer('alpha_bar', _alpha_bar(DIFFUSION_STEPS))
        self.register_buffer('latents', _fixed_latents(3))

    def distill_loss(self, context, target):
        """Train f(x_t, t) -> x0 for noised targets at random timesteps."""
        b = target.shape[0]
        x0 = target.reshape(b, TRAJ)
        ti = torch.randint(0, DIFFUSION_STEPS, (b,))
        ab = self.alpha_bar[ti].unsqueeze(-1)
        noise = torch.randn_like(x0)
        xt = ab.sqrt() * x0 + (1 - ab).sqrt() * noise
        pred = self.f(xt, context, (ti.float() / DIFFUSION_STEPS).unsqueeze(-1))
        return ((pred - x0) ** 2).mean()

    def forward(self, context):
        outs = []
        one = torch.ones(context.shape[0], 1)
        for k in range(NUM_CANDIDATES):
            z = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            outs.append(self.f(z, context, one).reshape(-1, 1, HORIZON, DIM))
        return torch.cat(outs, dim=1)


_LOSS = {
    'flow': lambda m, c, t: m.flow_loss(c, t),
    'diffusion': lambda m, c, t: m.diffusion_loss(c, t),
    'consistency': lambda m, c, t: m.distill_loss(c, t),
}


def build_planner(kind):
    """Instantiate a planner by family name: flow / diffusion / consistency."""
    if kind == 'flow':
        return FlowMatchingPlanner()
    if kind == 'diffusion':
        return DiffusionPlanner()
    if kind == 'consistency':
        return ConsistencyPlanner()
    raise ValueError('unknown planner kind: ' + kind)


def train_and_export(kind, path, num_samples=32, epochs=40, lr=0.01):
    """Train a generative planner family and export it to the ONNX seam contract."""
    torch.manual_seed(0)
    model = build_planner(kind)
    inputs, targets = make_synthetic_dataset(num_samples)
    expert = targets[:, 0, :, :]  # single expert trajectory per sample [N, H, 3]
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        loss = _LOSS[kind](model, inputs, expert)
        loss.backward()
        optimizer.step()

    model.eval()
    dummy = torch.zeros(1, 4)
    torch.onnx.export(
        model, dummy, path,
        input_names=['context'], output_names=['trajectories'],
        opset_version=18)
    # Inline weights into a single self-contained .onnx (no external .data
    # sidecar) so the model is a single deployable artifact for the backend.
    onnx.save_model(onnx.load(path), path, save_as_external_data=False)
    return float(loss.item())
