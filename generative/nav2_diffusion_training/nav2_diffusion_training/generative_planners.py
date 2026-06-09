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
Generative trajectory-model families for the TrajectoryModel ONNX seam.

Implements five goal-conditioned generative local planners that no surveyed work
open-sources for ROS 2 Nav2 ground robots: flow matching (single/few-step),
diffusion (DDIM), a one-step distilled (consistency-style) planner, a
transformer set-prediction planner (DETR-style learned query tokens, deterministic
single forward), and a recurrent (GRU) autoregressive rollout planner. Each maps a
context vector
[goal_x, goal_y, linear_speed, max_angular] to K candidate SE(2) trajectories and
exports to the C++ OnnxTrajectoryModel contract
context [1, 4] -> trajectories [1, K, H, 3]. The costmap-conditioned variants add a
[1, 1, S, S] egocentric patch input.

PyTorch is a heavy optional dependency, imported here (not from __init__); tests
skip when torch is unavailable.
"""

import math

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


class _CrossAttnBlock(nn.Module):
    """
    One pre-norm transformer decoder block (multi-head cross-attention + FFN).

    Implemented from primitives (matmul / softmax / layernorm / gelu) rather than
    ``nn.MultiheadAttention`` so the ONNX export is small and self-contained with no
    backend-specific attention ops.
    """

    def __init__(self, d, nhead):
        super().__init__()
        self.nhead = nhead
        self.dh = d // nhead
        self.q = nn.Linear(d, d)
        self.k = nn.Linear(d, d)
        self.v = nn.Linear(d, d)
        self.o = nn.Linear(d, d)
        self.n1 = nn.LayerNorm(d)
        self.n2 = nn.LayerNorm(d)
        self.ff = nn.Sequential(nn.Linear(d, 4 * d), nn.GELU(), nn.Linear(4 * d, d))

    def _attend(self, q, kv):
        b, nq, d = q.shape
        nk = kv.shape[1]
        h, dh = self.nhead, self.dh
        qh = self.q(q).reshape(b, nq, h, dh).transpose(1, 2)   # [b, h, nq, dh]
        kh = self.k(kv).reshape(b, nk, h, dh).transpose(1, 2)
        vh = self.v(kv).reshape(b, nk, h, dh).transpose(1, 2)
        att = (qh @ kh.transpose(-1, -2)) / math.sqrt(dh)
        out = (att.softmax(dim=-1) @ vh).transpose(1, 2).reshape(b, nq, d)
        return self.o(out)

    def forward(self, q, kv):
        q = q + self._attend(self.n1(q), kv)
        return q + self.ff(self.n2(q))


class _CostmapTokenizer(nn.Module):
    """
    Tokenize an egocentric costmap patch into spatial tokens for attention.

    A strided conv turns the COSTMAP_SIZE patch into a coarse feature grid whose
    cells become a token sequence with learned positional embeddings.
    """

    def __init__(self, d_model):
        super().__init__()
        self.proj = nn.Conv2d(1, d_model, kernel_size=4, stride=4)
        n = (COSTMAP_SIZE // 4) ** 2
        self.pos = nn.Parameter(torch.randn(1, n, d_model) * 0.1)

    def forward(self, costmap):
        x = self.proj(costmap)                       # [B, D, S/4, S/4]
        b, d, gh, gw = x.shape
        x = x.reshape(b, d, gh * gw).transpose(1, 2)  # [B, N, D]
        return x + self.pos


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


_TF_DIM = 32      # transformer token width
_TF_HEADS = 4
_TF_LAYERS = 2


class TransformerPlanner(nn.Module):
    """
    Context-only transformer set-prediction planner (no costmap).

    A DETR-style decoder: ``NUM_CANDIDATES`` learned query tokens cross-attend to a
    single context token and each decodes a full SE(2) trajectory in one forward
    pass. Multimodality comes from the distinct learned queries, not from sampling
    noise, so it is deterministic and single-step — the transformer member of the
    generative family, complementing flow / diffusion / consistency.
    """

    def __init__(self):
        super().__init__()
        self.ctx = nn.Linear(4, _TF_DIM)
        self.blocks = nn.ModuleList(
            [_CrossAttnBlock(_TF_DIM, _TF_HEADS) for _ in range(_TF_LAYERS)])
        self.queries = nn.Parameter(torch.randn(NUM_CANDIDATES, _TF_DIM) * 0.1)
        self.head = nn.Sequential(nn.LayerNorm(_TF_DIM), nn.Linear(_TF_DIM, TRAJ))

    def forward(self, context):
        b = context.shape[0]
        memory = self.ctx(context).unsqueeze(1)                  # [B, 1, D]
        q = self.queries.unsqueeze(0).expand(b, -1, -1)          # [B, K, D]
        for blk in self.blocks:
            q = blk(q, memory)
        return self.head(q).reshape(b, NUM_CANDIDATES, HORIZON, DIM)

    def recon_loss(self, context, target):
        """Direct regression of the K candidates onto the expert (set targets)."""
        return ((self(context) - target.unsqueeze(1)) ** 2).mean()


class CostmapTransformerPlanner(nn.Module):
    """
    Costmap+goal conditioned transformer set-prediction planner.

    The memory is a context token concatenated with costmap patch tokens; the K
    learned query tokens cross-attend over it and each decodes a full SE(2)
    trajectory in a single forward pass (no iterative sampling). This is the fourth
    model family on the OnnxTrajectoryModel contract — no surveyed work
    open-sources a transformer local trajectory planner integrated with Nav2.
    """

    def __init__(self):
        super().__init__()
        self.tok = _CostmapTokenizer(_TF_DIM)
        self.ctx = nn.Linear(4, _TF_DIM)
        self.blocks = nn.ModuleList(
            [_CrossAttnBlock(_TF_DIM, _TF_HEADS) for _ in range(_TF_LAYERS)])
        self.queries = nn.Parameter(torch.randn(NUM_CANDIDATES, _TF_DIM) * 0.1)
        self.head = nn.Sequential(nn.LayerNorm(_TF_DIM), nn.Linear(_TF_DIM, TRAJ))

    def _memory(self, context, costmap):
        tokens = self.tok(costmap)                               # [B, N, D]
        ctx = self.ctx(context).unsqueeze(1)                     # [B, 1, D]
        return torch.cat([ctx, tokens], dim=1)                  # [B, 1 + N, D]

    def forward(self, context, costmap):
        b = context.shape[0]
        memory = self._memory(context, costmap)
        q = self.queries.unsqueeze(0).expand(b, -1, -1)          # [B, K, D]
        for blk in self.blocks:
            q = blk(q, memory)
        return self.head(q).reshape(b, NUM_CANDIDATES, HORIZON, DIM)

    def recon_loss(self, context, costmap, target):
        """Direct regression of the K candidates onto the costmap-aware expert."""
        return ((self(context, costmap) - target.unsqueeze(1)) ** 2).mean()


_GRU_HID = 64     # recurrent rollout hidden width


def _gru_rollout(cell, head, seeds, cond):
    """
    Autoregressively roll out ``NUM_CANDIDATES`` SE(2) trajectories with a GRU.

    Each candidate ``k`` gets a distinct conditioning ``cond + seeds[k]`` that both
    initialises the hidden state and is fed at every step alongside the previously
    emitted point, so the K rollouts stay distinct (multimodality from the learned
    seeds, like the transformer's learned queries). The ``HORIZON``/``K`` python
    loops unroll into a static graph, so the GRU exports cleanly to ONNX.
    """
    b = cond.shape[0]
    outs = []
    for k in range(NUM_CANDIDATES):
        cond_k = cond + seeds[k].unsqueeze(0)           # [B, _GRU_HID]
        h = cond_k
        prev = torch.zeros(b, DIM, device=cond.device, dtype=cond.dtype)
        steps = []
        for _ in range(HORIZON):
            h = cell(torch.cat([prev, cond_k], dim=-1), h)
            prev = head(h)
            steps.append(prev)
        outs.append(torch.stack(steps, dim=1).unsqueeze(1))  # [B, 1, H, DIM]
    return torch.cat(outs, dim=1)                       # [B, K, H, DIM]


class RecurrentRolloutPlanner(nn.Module):
    """
    Context-only recurrent (GRU) autoregressive rollout planner.

    Instead of proposing a whole trajectory at once (flow / diffusion / consistency
    / transformer), a GRU emits the SE(2) points one step at a time, feeding the
    previous point back in — the sequential inductive bias of a world-model-style
    rollout. The fifth member of the generative family.
    """

    def __init__(self):
        super().__init__()
        self.ctx = nn.Linear(4, _GRU_HID)
        self.seeds = nn.Parameter(torch.randn(NUM_CANDIDATES, _GRU_HID) * 0.1)
        self.cell = nn.GRUCell(DIM + _GRU_HID, _GRU_HID)
        self.head = nn.Linear(_GRU_HID, DIM)

    def forward(self, context):
        return _gru_rollout(self.cell, self.head, self.seeds, self.ctx(context))

    def recon_loss(self, context, target):
        """Direct regression of the K rolled-out candidates onto the expert."""
        return ((self(context) - target.unsqueeze(1)) ** 2).mean()


class CostmapRecurrentPlanner(nn.Module):
    """
    Costmap+goal conditioned recurrent (GRU) autoregressive rollout planner.

    The costmap patch is encoded to an embedding that, with the context, forms the
    rollout conditioning; a GRU then emits the trajectory one point at a time. The
    fifth model family on the OnnxTrajectoryModel contract — no surveyed work
    open-sources a recurrent rollout local planner integrated with Nav2.
    """

    def __init__(self):
        super().__init__()
        self.encoder = _CostmapEncoder()
        self.cond = nn.Linear(4 + COSTMAP_EMBED, _GRU_HID)
        self.seeds = nn.Parameter(torch.randn(NUM_CANDIDATES, _GRU_HID) * 0.1)
        self.cell = nn.GRUCell(DIM + _GRU_HID, _GRU_HID)
        self.head = nn.Linear(_GRU_HID, DIM)

    def _conditioning(self, context, costmap):
        return self.cond(torch.cat([context, self.encoder(costmap)], dim=-1))

    def forward(self, context, costmap):
        cond = self._conditioning(context, costmap)
        return _gru_rollout(self.cell, self.head, self.seeds, cond)

    def recon_loss(self, context, costmap, target):
        """Direct regression of the K rolled-out candidates onto the expert."""
        return ((self(context, costmap) - target.unsqueeze(1)) ** 2).mean()


def _expert_trajectory(gx, gy, side, speed):
    """
    Build a ~1 s pure-pursuit expert arc toward the carrot with an avoidance bow.

    A ~1 s expert that *pursues* the carrot (gx, gy) along a constant-curvature arc
    (pure-pursuit), travelling ``speed`` * horizon metres, plus a half-sine lateral
    bow away from a one-sided obstacle (side > 0 = obstacle on +y -> bow to -y).

    The arc turns the robot toward the carrot, so in closed loop a lateral offset
    from the plan (which moves the carrot to the side) is actively corrected — the
    key to staying on track over a long run. side == 0 is the clear case (pure
    pursuit, no bow). yaw is the path tangent so the extracted command turns
    correctly.
    """
    s_total = speed * HORIZON * 0.1
    den = gx * gx + gy * gy
    kappa = (2.0 * gy / den) if den > 1e-6 else 0.0   # pure-pursuit curvature
    pts = []
    for h in range(HORIZON):
        s = s_total * (h + 1) / HORIZON
        if abs(kappa) < 1e-4:
            x, y, yaw = s, 0.0, 0.0
        else:
            r = 1.0 / kappa
            theta = s * kappa
            x = r * math.sin(theta)
            y = r * (1.0 - math.cos(theta))
            yaw = theta
        t = (h + 1) / HORIZON
        veer = -side * 0.20 * (4.0 * t * (1.0 - t))    # 0 at both ends
        x += -math.sin(yaw) * veer                     # bow perpendicular to heading
        y += math.cos(yaw) * veer
        pts.append([x, y, yaw])
    for h in range(HORIZON):                           # yaw = path tangent (incl. bow)
        if h == 0:
            ref_a, ref_b = pts[0], pts[1]
        elif h + 1 >= HORIZON:
            ref_a, ref_b = pts[h - 1], pts[h]
        else:
            ref_a, ref_b = pts[h - 1], pts[h + 1]
        pts[h][2] = math.atan2(ref_b[1] - ref_a[1], ref_b[0] - ref_a[0])
    return pts


def make_costmap_dataset(num_samples):
    """
    Synthetic costmap patches with a side obstacle and a carrot-directed expert.

    Obstacle on the left (+y, low cols — matching cropEgocentricPatch) -> expert
    veers right (-y), and vice versa, so the model must read the costmap to choose
    the avoidance direction. The carrot (context goal_x/goal_y) is *varied* in
    distance and bearing so the closed-loop controller stays in distribution as its
    heading drifts; the expert heads toward the carrot with a lateral avoidance
    bow. Obstacle row-band / column width vary, every configuration is a mirrored
    +y/-y pair, and clear (no-obstacle) samples anchor the unconditioned behaviour
    to driving straight at the carrot.
    """
    contexts = []
    patches = []
    targets = []
    s = COSTMAP_SIZE
    row_bands = [(s // 4, s // 2), (s // 3, 2 * s // 3), (s // 6, s // 2)]
    widths = [s // 2, s // 3, 2 * s // 5]   # columns the obstacle spans from its edge
    goals = [(1.0, 0.0), (0.9, 0.0), (1.1, 0.0), (1.0, 0.2), (1.0, -0.2),
             (1.0, 0.4), (1.0, -0.4), (0.9, 0.3), (0.9, -0.3), (1.1, 0.15),
             (1.1, -0.15)]
    speed = 0.3
    i = 0
    while len(contexts) < num_samples:
        gx, gy = goals[i % len(goals)]
        rb = row_bands[(i // 2) % len(row_bands)]
        w = widths[(i // 2) % len(widths)]
        for side in (1.0, -1.0):            # +1 obstacle on left (+y / low cols)
            if len(contexts) >= num_samples:
                break
            patch = torch.zeros(1, s, s)
            c0, c1 = (0, w) if side > 0 else (s - w, s)
            patch[:, rb[0]:rb[1], c0:c1] = 1.0
            contexts.append([gx, gy, speed, 1.0])
            patches.append(patch)
            targets.append(_expert_trajectory(gx, gy, side, speed))
        if len(contexts) < num_samples:     # clear -> straight at the carrot
            contexts.append([gx, gy, speed, 1.0])
            patches.append(torch.zeros(1, s, s))
            targets.append(_expert_trajectory(gx, gy, 0.0, speed))
        i += 1
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.stack(patches),
        torch.tensor(targets, dtype=torch.float32),
    )


_COSTMAP_BUILD = {
    'flow': CostmapFlowPlanner,
    'diffusion': CostmapDiffusionPlanner,
    'consistency': CostmapConsistencyPlanner,
    'transformer': CostmapTransformerPlanner,
    'recurrent': CostmapRecurrentPlanner,
}
_COSTMAP_LOSS = {
    'flow': lambda m, c, cm, t: m.flow_loss(c, cm, t),
    'diffusion': lambda m, c, cm, t: m.diffusion_loss(c, cm, t),
    'consistency': lambda m, c, cm, t: m.distill_loss(c, cm, t),
    'transformer': lambda m, c, cm, t: m.recon_loss(c, cm, t),
    'recurrent': lambda m, c, cm, t: m.recon_loss(c, cm, t),
}


def train_and_export_costmap(
        path, kind='flow', num_samples=32, epochs=60, lr=0.01,
        steps=None, sample_weight=0.0, device=None):
    """
    Train a costmap planner (flow/diffusion/consistency/transformer/recurrent).

    Exports the 2-input ONNX seam (``context [1,4]`` + ``costmap [1,1,S,S]`` ->
    ``[1,K,H,3]``).

    ``steps`` overrides the flow integration steps (more = smoother samples).
    ``sample_weight`` > 0 adds a direct MSE between the *sampled* trajectory and the
    expert target on top of the generative loss, which yields a smooth, ordered
    output whose per-step speeds stay within kinematic limits (so the controller's
    safety gate accepts it). Both default to the previous behaviour.

    ``device`` selects the *training* device (e.g. ``'cuda'``); ``None`` keeps the
    CPU path. The model is always moved back to CPU for the ONNX export, so the
    exported artifact is portable regardless of where it was trained.
    """
    torch.manual_seed(0)
    dev = torch.device(device) if device is not None else torch.device('cpu')
    if kind == 'flow' and steps is not None:
        model = CostmapFlowPlanner(steps=steps)
    else:
        model = _COSTMAP_BUILD[kind]()
    model.to(dev)
    context, costmap, target = make_costmap_dataset(num_samples)
    context, costmap, target = context.to(dev), costmap.to(dev), target.to(dev)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        loss = _COSTMAP_LOSS[kind](model, context, costmap, target)
        if sample_weight > 0.0:
            out = model(context, costmap)                       # [B, K, H, 3]
            loss = loss + sample_weight * ((out - target.unsqueeze(1)) ** 2).mean()
        loss.backward()
        optimizer.step()

    model.eval().to('cpu')
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
    'transformer': lambda m, c, t: m.recon_loss(c, t),
    'recurrent': lambda m, c, t: m.recon_loss(c, t),
}


def build_planner(kind):
    """Instantiate a planner by family: flow/diffusion/consistency/transformer/recurrent."""
    if kind == 'flow':
        return FlowMatchingPlanner()
    if kind == 'diffusion':
        return DiffusionPlanner()
    if kind == 'consistency':
        return ConsistencyPlanner()
    if kind == 'transformer':
        return TransformerPlanner()
    if kind == 'recurrent':
        return RecurrentRolloutPlanner()
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
