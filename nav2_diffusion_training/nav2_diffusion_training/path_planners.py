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

import math

from nav2_diffusion_training.generative_planners import _CrossAttnBlock

import onnx

import torch
from torch import nn

PATH_K = 5            # candidate paths proposed
PATH_H = 12           # waypoints per path
PATH_DIM = 2          # x, y (aligned frame)
# Goal-aligned costmap patch fed to the costmap-conditioned path model. The
# patch covers a fixed physical window ahead of the start: aligned x in
# [0, PATCH_FWD], aligned y in [-PATCH_HALF, PATCH_HALF], on a SIZE x SIZE grid
# (row -> forward x, col -> lateral y). The C++ backend resamples the real
# global costmap into this same window, so the learned avoidance transfers.
PATH_COSTMAP_SIZE = 24
PATCH_FWD = 6.0       # metres ahead covered by the patch
PATCH_HALF = 3.0      # metres to each side covered by the patch
PATH_EMBED = 16
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


class _PathCostmapEncoder(nn.Module):
    """Small CNN encoding a goal-aligned costmap patch to an embedding vector."""

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 8, 3, stride=2, padding=1), nn.SiLU(),
            nn.Conv2d(8, 16, 3, stride=2, padding=1), nn.SiLU(),
            nn.AdaptiveAvgPool2d(1), nn.Flatten(),
            nn.Linear(16, PATH_EMBED), nn.SiLU(),
        )

    def forward(self, costmap):
        return self.net(costmap)


class CostmapPathFlowPlanner(nn.Module):
    """
    Costmap+goal conditioned generative global-path planner (Nav2 Mode B).

    Reads a goal-aligned costmap patch and proposes K start->goal paths that bias
    toward the obstacle-free side. This is the global (Mode B) analogue of the
    costmap-conditioned local planner; no surveyed work open-sources it for Nav2.
    """

    def __init__(self, steps=4):
        super().__init__()
        self.encoder = _PathCostmapEncoder()
        self.field = nn.Sequential(
            nn.Linear(PATH_VEC + CTX_DIM + PATH_EMBED + 1, 128), nn.SiLU(),
            nn.Linear(128, 128), nn.SiLU(),
            nn.Linear(128, PATH_VEC),
        )
        self.steps = steps
        self.register_buffer('latents', _fixed_latents(13))

    def velocity(self, x, context, embed, t):
        return self.field(torch.cat([x, context, embed, t], dim=-1))

    def flow_loss(self, context, costmap, target):
        """Conditional flow-matching loss conditioned on the costmap patch."""
        b = target.shape[0]
        embed = self.encoder(costmap)
        x1 = target.reshape(b, PATH_VEC)
        x0 = torch.randn_like(x1)
        t = torch.rand(b, 1)
        xt = (1.0 - t) * x0 + t * x1
        pred = self.velocity(xt, context, embed, t)
        return ((pred - (x1 - x0)) ** 2).mean()

    def forward(self, context, costmap):
        embed = self.encoder(costmap)
        outs = []
        dt = 1.0 / self.steps
        for k in range(PATH_K):
            x = self.latents[k].unsqueeze(0).expand(context.shape[0], -1)
            for i in range(self.steps):
                t = torch.full((x.shape[0], 1), i * dt)
                x = x + self.velocity(x, context, embed, t) * dt
            outs.append(x.reshape(-1, 1, PATH_H, PATH_DIM))
        return torch.cat(outs, dim=1)


_PATH_TF_DIM = 32
_PATH_TF_HEADS = 4
_PATH_TF_LAYERS = 2
_PATH_FAN_W = 0.4     # half-width [m] of the lateral candidate fan (set-prediction diversity)


class _PathCostmapTokenizer(nn.Module):
    """Tokenize the goal-aligned costmap patch into spatial tokens for attention."""

    def __init__(self, d_model):
        super().__init__()
        self.proj = nn.Conv2d(1, d_model, kernel_size=4, stride=4)
        n = (PATH_COSTMAP_SIZE // 4) ** 2
        self.pos = nn.Parameter(torch.randn(1, n, d_model) * 0.1)

    def forward(self, costmap):
        x = self.proj(costmap)                       # [B, D, S/4, S/4]
        b, d, gh, gw = x.shape
        x = x.reshape(b, d, gh * gw).transpose(1, 2)  # [B, N, D]
        return x + self.pos


class CostmapPathTransformerPlanner(nn.Module):
    """
    Costmap+goal conditioned transformer set-prediction global-path planner.

    The Mode B (global path) analogue of the transformer trajectory model: the
    goal-aligned costmap patch is tokenized (strided conv + learned positions) and
    prepended with a context token; K learned query tokens cross-attend to that
    memory and each decodes a full start->goal path in one deterministic forward
    pass. Attention over explicit costmap tokens (vs the flow model's 16-d CNN
    embedding) lets it *aim* its proposals at an off-centre slot, which the flow
    model cannot (docs/generative_limits.md).

    The K candidates are trained as a small **lateral fan** around the expert
    (candidate k targets ``expert.y + fan[k]``, ``fan`` a fixed linspace over
    ``+/-_PATH_FAN_W``). Without the fan the queries collapse to one path, so a
    single clipped proposal means the footprint validator finds *no* clear path;
    the fan gives the validator a spread of options around the aimed route (the
    flow model gets this spread for free from its K fixed latents).
    """

    def __init__(self):
        super().__init__()
        self.tok = _PathCostmapTokenizer(_PATH_TF_DIM)
        self.ctx = nn.Linear(CTX_DIM, _PATH_TF_DIM)
        self.blocks = nn.ModuleList(
            [_CrossAttnBlock(_PATH_TF_DIM, _PATH_TF_HEADS) for _ in range(_PATH_TF_LAYERS)])
        self.queries = nn.Parameter(torch.randn(PATH_K, _PATH_TF_DIM) * 0.1)
        self.head = nn.Sequential(
            nn.LayerNorm(_PATH_TF_DIM), nn.Linear(_PATH_TF_DIM, PATH_VEC))
        self.register_buffer('fan', torch.linspace(-_PATH_FAN_W, _PATH_FAN_W, PATH_K))

    def forward(self, context, costmap):
        b = context.shape[0]
        tokens = self.tok(costmap)                          # [B, N, D]
        ctx = self.ctx(context).unsqueeze(1)                # [B, 1, D]
        memory = torch.cat([ctx, tokens], dim=1)            # [B, 1 + N, D]
        q = self.queries.unsqueeze(0).expand(b, -1, -1)     # [B, K, D]
        for blk in self.blocks:
            q = blk(q, memory)
        return self.head(q).reshape(b, PATH_K, PATH_H, PATH_DIM)

    def recon_loss(self, context, costmap, target):
        """Regress the K candidates onto a lateral fan around the routing expert."""
        out = self(context, costmap)                        # [B, K, H, 2]
        tgt_x = target[..., 0].unsqueeze(1).expand(-1, PATH_K, -1)
        tgt_y = target[..., 1].unsqueeze(1) + self.fan.view(1, PATH_K, 1)
        tgt = torch.stack([tgt_x, tgt_y], dim=-1)           # [B, K, H, 2]
        return ((out - tgt) ** 2).mean()


def _aligned_patch(side, inner=0.0, x_lo=1.5, x_hi=4.0):
    """
    Build a patch (row->fwd x, col->lateral y) with an obstacle on one side.

    side > 0 places the obstacle on the +y (left) half ahead, so the gap (and
    the expert) is on the -y (right) side, and vice versa. ``inner`` is the
    lateral distance (m) from the centre line at which the obstacle band starts,
    and ``x_lo``/``x_hi`` are the forward extent (m) of the band. Varying these
    teaches the model to respond to partial / weaker obstacle signals (the real
    costmap rarely fills a whole half), which is what the C++ backend resamples
    from the live costmap.
    """
    s = PATH_COSTMAP_SIZE
    patch = torch.zeros(1, s, s)
    r0 = max(0, int(s * x_lo / PATCH_FWD))
    r1 = min(s, int(s * x_hi / PATCH_FWD))
    for c in range(s):
        ay = -PATCH_HALF + 2.0 * PATCH_HALF * (c + 0.5) / s
        # Signed lateral coordinate on the obstacle side; occupy [inner, edge].
        if inner < side * ay < PATCH_HALF:
            patch[:, r0:r1, c] = 1.0
    return patch


def make_costmap_path_dataset(num_samples):
    """
    Goal-aligned paths that veer toward the obstacle-free side of the patch.

    Each sample has a one-sided obstacle ahead; the expert path bows to the open
    side, so the model must read the costmap to choose the avoidance direction.
    Obstacle width (``inner``) and forward extent vary, and every configuration is
    emitted as a mirrored +y/-y pair, so the learned response is symmetric and
    robust to partial obstacle signals rather than memorising one full half.
    """
    contexts = []
    patches = []
    targets = []
    # Deterministic grids of obstacle width (inner edge) and forward extent so the
    # model sees strong and weak / partial signals, not one canonical full half.
    inners = [0.0, 0.3, 0.6, 1.0]
    spans = [(1.5, 4.0), (1.0, 3.0), (2.0, 4.5), (1.5, 3.0)]
    i = 0
    while len(contexts) < num_samples:
        d = 3.0 + 2.0 * ((i * 5) % 7) / 6.0          # goal distance 3..5 m
        inner = inners[i % len(inners)]
        x_lo, x_hi = spans[(i // 2) % len(spans)]
        # Emit a mirrored +y / -y pair for every configuration -> symmetric data.
        for side in (1.0, -1.0):
            if len(contexts) >= num_samples:
                break
            gap_sign = -side                          # veer to the open side
            rows = []
            for h in range(PATH_H):
                t = h / (PATH_H - 1)
                bump = t * (1.0 - t) * 4.0            # 0 at both ends
                rows.append([t * d, gap_sign * 0.9 * bump])
            contexts.append([d, 0.0])
            patches.append(_aligned_patch(side, inner=inner, x_lo=x_lo, x_hi=x_hi))
            targets.append(rows)
        # Anchor the unconditioned (clear) behaviour to the straight line so the
        # model has no built-in left/right bias; the K latents still fan the
        # proposals out around it.
        if len(contexts) < num_samples:
            contexts.append([d, 0.0])
            patches.append(torch.zeros(1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE))
            targets.append([[(h / (PATH_H - 1)) * d, 0.0] for h in range(PATH_H)])
        i += 1
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.stack(patches),
        torch.tensor(targets, dtype=torch.float32),
    )


def _gap_patch(slot_y, x_lo, x_hi, slot_hw):
    """Build a wall spanning the patch width at forward band [x_lo, x_hi] with one slot.

    The wall fills every column except a gap of half-width ``slot_hw`` centred at
    lateral ``slot_y`` (metres). Routing through the slot — not picking a free side
    — is the off-centre-gap problem the flow model could not solve.
    """
    s = PATH_COSTMAP_SIZE
    patch = torch.zeros(1, s, s)
    r0 = max(0, int(s * x_lo / PATCH_FWD))
    r1 = min(s, int(s * x_hi / PATCH_FWD))
    for c in range(s):
        ay = -PATCH_HALF + 2.0 * PATCH_HALF * (c + 0.5) / s
        if abs(ay - slot_y) > slot_hw:        # wall everywhere except the slot
            patch[:, r0:r1, c] = 1.0
    return patch


def make_costmap_path_gap_dataset(num_samples):
    """
    Off-centre-gap routing data: a wall across the path with a single off-centre slot.

    Unlike the one-sided-obstacle data, the expert must *route through the slot*
    (detour to the slot's lateral offset at the wall, then return toward the goal),
    so the model has to localize the gap, not just pick a free side. Slot offset and
    side, wall position and slot width vary; every slot is emitted as a mirrored
    +y/-y pair. A few clear samples anchor the straight-line behaviour.
    """
    contexts = []
    patches = []
    targets = []
    slot_offsets = [1.2, 1.6, 2.0]            # |lateral| offset of the slot [m]
    # Wall forward-extent (m). Includes a band centred near 2 m so the training
    # distribution covers the catalog off-centre-gap scenario (wall at aligned x~2,
    # goal distance ~4) rather than only the 2.5-3.3 m bands used before.
    spans = [(1.6, 2.4), (2.5, 3.3), (2.2, 3.0), (2.8, 3.6)]
    slot_hws = [0.5, 0.6, 0.45, 0.5]          # slot half-width [m]
    i = 0
    while len(contexts) < num_samples:
        d = 4.0 + 1.5 * ((i * 3) % 5) / 4.0   # goal distance 4.0..5.5 m
        off = slot_offsets[i % len(slot_offsets)]
        x_lo, x_hi = spans[(i // 2) % len(spans)]
        slot_hw = slot_hws[(i // 2) % len(slot_hws)]
        x_wall = 0.5 * (x_lo + x_hi)
        t_wall = min(0.9, max(0.1, x_wall / d))
        sigma = 0.22
        for side in (1.0, -1.0):              # slot on +y or -y
            if len(contexts) >= num_samples:
                break
            slot_y = side * off
            rows = []
            for h in range(PATH_H):
                t = h / (PATH_H - 1)
                # Gaussian detour peaking at the slot offset as the path crosses
                # the wall, ~0 at start and goal.
                bump = math.exp(-((t - t_wall) / sigma) ** 2)
                rows.append([t * d, slot_y * bump])
            contexts.append([d, 0.0])
            patches.append(_gap_patch(slot_y, x_lo, x_hi, slot_hw))
            targets.append(rows)
        if len(contexts) < num_samples:
            contexts.append([d, 0.0])
            patches.append(torch.zeros(1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE))
            targets.append([[(h / (PATH_H - 1)) * d, 0.0] for h in range(PATH_H)])
        i += 1
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.stack(patches),
        torch.tensor(targets, dtype=torch.float32),
    )


def _path_dataset(dataset, num_samples):
    """Select / combine the path datasets: 'side', 'gap', or 'both'."""
    if dataset == 'side':
        return make_costmap_path_dataset(num_samples)
    if dataset == 'gap':
        return make_costmap_path_gap_dataset(num_samples)
    if dataset == 'both':
        half = num_samples // 2
        a = make_costmap_path_dataset(num_samples - half)
        b = make_costmap_path_gap_dataset(half)
        return tuple(torch.cat([a[i], b[i]], dim=0) for i in range(3))
    raise ValueError('unknown path dataset: ' + dataset)


def train_and_export_costmap_path(path, num_samples=96, epochs=400, lr=0.01, steps=4,
                                  kind='flow', dataset='side', device=None):
    """Train a costmap-conditioned Mode B path planner and export a 2-input ONNX.

    ``kind`` selects the family: ``'flow'`` (CostmapPathFlowPlanner) or
    ``'transformer'`` (CostmapPathTransformerPlanner). ``dataset`` selects the
    training data: ``'side'`` (one-sided obstacle, the default / shipped behaviour),
    ``'gap'`` (off-centre slot routing), or ``'both'``. ``device`` selects the
    training device (e.g. ``'cuda'``); the model is always exported on CPU so the
    artifact is portable.
    """
    torch.manual_seed(0)
    dev = torch.device(device) if device is not None else torch.device('cpu')
    if kind == 'flow':
        model = CostmapPathFlowPlanner(steps=steps)
    elif kind == 'transformer':
        model = CostmapPathTransformerPlanner()
    else:
        raise ValueError('unknown path planner kind: ' + kind)
    model.to(dev)
    context, costmap, target = _path_dataset(dataset, num_samples)
    context, costmap, target = context.to(dev), costmap.to(dev), target.to(dev)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        if kind == 'flow':
            loss = model.flow_loss(context, costmap, target)
            out = model(context[:16], costmap[:16])         # [B, K, H, 2]
        else:
            loss = model.recon_loss(context, costmap, target)
            out = model(context[:16], costmap[:16])
        jerk = out[:, :, 2:, :] - 2 * out[:, :, 1:-1, :] + out[:, :, :-2, :]
        loss = loss + 2.0 * (jerk ** 2).mean()              # smoothness
        loss.backward()
        optimizer.step()

    model.eval().to('cpu')
    dummy_ctx = torch.zeros(1, CTX_DIM)
    dummy_map = torch.zeros(1, 1, PATH_COSTMAP_SIZE, PATH_COSTMAP_SIZE)
    torch.onnx.export(
        model, (dummy_ctx, dummy_map), path,
        input_names=['context', 'costmap'], output_names=['paths'],
        opset_version=18)
    onnx.save_model(onnx.load(path), path, save_as_external_data=False)
    return float(loss.item())
