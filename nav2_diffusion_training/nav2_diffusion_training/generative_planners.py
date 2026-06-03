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
COSTMAP_SIZE = 32            # egocentric costmap patch side length [cells]
COSTMAP_EMBED = 16
# DDIM x0 thresholding bound [m]. The cosine schedule's final alpha_bar is ~0,
# so x0 = (x - sqrt(1-ab) * eps) / sqrt(ab) divides by ~0 and amplifies any eps
# error into a non-physical trajectory. Local trajectories live well within a
# couple of metres, so clamping x0 each step keeps sampling numerically stable
# (standard static thresholding) without distorting the valid range.
X0_CLAMP = 3.0


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
            x0 = ((x - (1 - ab).sqrt() * eps) / ab.sqrt()).clamp(-X0_CLAMP, X0_CLAMP)
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


class _CostmapEncoder(nn.Module):
    """Small CNN encoding an egocentric costmap patch to an embedding vector."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 8, 3, stride=2, padding=1), nn.SiLU(),
            nn.Conv2d(8, 16, 3, stride=2, padding=1), nn.SiLU(),
            nn.AdaptiveAvgPool2d(1), nn.Flatten(),
            nn.Linear(16, COSTMAP_EMBED), nn.SiLU(),
        )

    def forward(self, costmap):
        return self.net(costmap)


class CostmapFlowPlanner(nn.Module):
    """Costmap+goal conditioned flow matching (the surveyed OSS gap for Nav2)."""

    def __init__(self, steps=2):
        super().__init__()
        self.encoder = _CostmapEncoder()
        self.field = nn.Sequential(
            nn.Linear(TRAJ + 4 + COSTMAP_EMBED + 1, 128), nn.SiLU(),
            nn.Linear(128, 128), nn.SiLU(),
            nn.Linear(128, TRAJ),
        )
        self.steps = steps
        self.register_buffer('latents', _fixed_latents(7))

    def velocity(self, x, context, embed, t):
        return self.field(torch.cat([x, context, embed, t], dim=-1))

    def flow_loss(self, context, costmap, target):
        """Conditional flow-matching loss conditioned on the costmap patch."""
        b = target.shape[0]
        embed = self.encoder(costmap)
        x1 = target.reshape(b, TRAJ)
        x0 = torch.randn_like(x1)
        t = torch.rand(b, 1)
        xt = (1.0 - t) * x0 + t * x1
        pred = self.velocity(xt, context, embed, t)
        return ((pred - (x1 - x0)) ** 2).mean()

    def forward(self, context, costmap):
        embed = self.encoder(costmap)
        outs = []
        dt = 1.0 / self.steps
        for k in range(NUM_CANDIDATES):
            x = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            for i in range(self.steps):
                t = torch.full((x.shape[0], 1), i * dt)
                x = x + self.velocity(x, context, embed, t) * dt
            outs.append(x.reshape(-1, 1, HORIZON, DIM))
        return torch.cat(outs, dim=1)


def _cond_field():
    """Conditioned trajectory net: (traj + context + costmap embed + time) -> traj."""
    return nn.Sequential(
        nn.Linear(TRAJ + 4 + COSTMAP_EMBED + 1, 128), nn.SiLU(),
        nn.Linear(128, 128), nn.SiLU(),
        nn.Linear(128, TRAJ),
    )


class CostmapDiffusionPlanner(nn.Module):
    """Costmap+goal conditioned diffusion planner (DDPM train, DDIM sample)."""

    def __init__(self, sample_steps=4):
        super().__init__()
        self.encoder = _CostmapEncoder()
        self.eps = _cond_field()
        self.sample_steps = sample_steps
        self.register_buffer('alpha_bar', _alpha_bar(DIFFUSION_STEPS))
        self.register_buffer('latents', _fixed_latents(8))

    def diffusion_loss(self, context, costmap, target):
        """Predict the added noise, conditioned on the costmap patch."""
        b = target.shape[0]
        embed = self.encoder(costmap)
        x0 = target.reshape(b, TRAJ)
        ti = torch.randint(0, DIFFUSION_STEPS, (b,))
        ab = self.alpha_bar[ti].unsqueeze(-1)
        noise = torch.randn_like(x0)
        xt = ab.sqrt() * x0 + (1 - ab).sqrt() * noise
        t = (ti.float() / DIFFUSION_STEPS).unsqueeze(-1)
        pred = self.eps(torch.cat([xt, context, embed, t], dim=-1))
        return ((pred - noise) ** 2).mean()

    def forward(self, context, costmap):
        embed = self.encoder(costmap)
        idx = torch.linspace(DIFFUSION_STEPS - 1, 0, self.sample_steps).long()
        outs = []
        for k in range(NUM_CANDIDATES):
            x = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            for j in range(self.sample_steps):
                i = int(idx[j])
                ab = self.alpha_bar[i]
                t = torch.full((x.shape[0], 1), float(i) / DIFFUSION_STEPS)
                eps = self.eps(torch.cat([x, context, embed, t], dim=-1))
                x0 = ((x - (1 - ab).sqrt() * eps) / ab.sqrt()).clamp(-X0_CLAMP, X0_CLAMP)
                if j + 1 < self.sample_steps:
                    ab_n = self.alpha_bar[int(idx[j + 1])]
                    x = ab_n.sqrt() * x0 + (1 - ab_n).sqrt() * eps
                else:
                    x = x0
            outs.append(x.reshape(-1, 1, HORIZON, DIM))
        return torch.cat(outs, dim=1)


class CostmapConsistencyPlanner(nn.Module):
    """Costmap+goal conditioned one-step distilled (consistency-style) planner."""

    def __init__(self):
        super().__init__()
        self.encoder = _CostmapEncoder()
        self.f = _cond_field()
        self.register_buffer('alpha_bar', _alpha_bar(DIFFUSION_STEPS))
        self.register_buffer('latents', _fixed_latents(9))

    def distill_loss(self, context, costmap, target):
        """Map a noised target back to x0, conditioned on the costmap patch."""
        b = target.shape[0]
        embed = self.encoder(costmap)
        x0 = target.reshape(b, TRAJ)
        ti = torch.randint(0, DIFFUSION_STEPS, (b,))
        ab = self.alpha_bar[ti].unsqueeze(-1)
        noise = torch.randn_like(x0)
        xt = ab.sqrt() * x0 + (1 - ab).sqrt() * noise
        t = (ti.float() / DIFFUSION_STEPS).unsqueeze(-1)
        pred = self.f(torch.cat([xt, context, embed, t], dim=-1))
        return ((pred - x0) ** 2).mean()

    def forward(self, context, costmap):
        embed = self.encoder(costmap)
        one = torch.ones(context.shape[0], 1)
        outs = []
        for k in range(NUM_CANDIDATES):
            z = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            outs.append(
                self.f(torch.cat([z, context, embed, one], dim=-1)).reshape(-1, 1, HORIZON, DIM))
        return torch.cat(outs, dim=1)


def make_costmap_dataset(num_samples):
    """
    Synthetic costmap patches with a side obstacle and an avoidance expert.

    Obstacle on the left -> expert veers right and vice versa, so the model must
    read the costmap to choose the avoidance direction.
    """
    contexts = []
    patches = []
    targets = []
    for i in range(num_samples):
        side = 1.0 if (i % 2 == 0) else -1.0  # +1 obstacle on left (+y)
        patch = torch.zeros(1, COSTMAP_SIZE, COSTMAP_SIZE)
        col0 = 0 if side > 0 else COSTMAP_SIZE // 2
        patch[:, COSTMAP_SIZE // 3:2 * COSTMAP_SIZE // 3, col0:col0 + COSTMAP_SIZE // 2] = 1.0
        rows = []
        for h in range(HORIZON):
            fwd = 0.3 * (h + 1) * 0.1
            lat = -side * 0.15 * (h + 1) / HORIZON  # veer away from the obstacle
            rows.append([fwd, lat, -side * 0.05])
        contexts.append([1.0, 0.0, 0.3, 1.0])
        patches.append(patch)
        targets.append(rows)
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.stack(patches),
        torch.tensor(targets, dtype=torch.float32),
    )


_COSTMAP_BUILD = {
    'flow': CostmapFlowPlanner,
    'diffusion': CostmapDiffusionPlanner,
    'consistency': CostmapConsistencyPlanner,
}
_COSTMAP_LOSS = {
    'flow': lambda m, c, cm, t: m.flow_loss(c, cm, t),
    'diffusion': lambda m, c, cm, t: m.diffusion_loss(c, cm, t),
    'consistency': lambda m, c, cm, t: m.distill_loss(c, cm, t),
}


def train_and_export_costmap(path, kind='flow', num_samples=32, epochs=60, lr=0.01):
    """Train a costmap-conditioned planner (flow/diffusion/consistency) -> 2-input ONNX."""
    torch.manual_seed(0)
    model = _COSTMAP_BUILD[kind]()
    context, costmap, target = make_costmap_dataset(num_samples)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        loss = _COSTMAP_LOSS[kind](model, context, costmap, target)
        loss.backward()
        optimizer.step()

    model.eval()
    dummy_ctx = torch.zeros(1, 4)
    dummy_map = torch.zeros(1, 1, COSTMAP_SIZE, COSTMAP_SIZE)
    torch.onnx.export(
        model, (dummy_ctx, dummy_map), path,
        input_names=['context', 'costmap'], output_names=['trajectories'],
        opset_version=18)
    onnx.save_model(onnx.load(path), path, save_as_external_data=False)
    return float(loss.item())


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
