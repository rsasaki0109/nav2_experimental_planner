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


_PATH_TF_DIM = 64
_PATH_TF_HEADS = 8
_PATH_TF_LAYERS = 3
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


_PATH_GRU_HID = 64


class CostmapPathRecurrentPlanner(nn.Module):
    """
    Costmap+goal conditioned GRU-rollout set-prediction global-path planner (Mode B).

    The Mode B (global path) analogue of the recurrent trajectory model: the
    goal-aligned costmap patch is encoded to a conditioning vector and a GRU emits
    each path one waypoint at a time, feeding the previous point back in, so the
    sequential inductive bias matches a path's waypoint-by-waypoint structure
    (vs the transformer's one-shot set decode and the flow model's iterative
    denoising). K learned seed vectors give the candidates distinct initial
    states; like the transformer they are trained as a lateral fan around the
    routing expert (candidate k targets ``expert.y + fan[k]``) so the footprint
    validator gets a spread of proposals instead of one collapsed path. Static
    PATH_H / PATH_K python loops unroll into an ONNX-clean graph (no custom ops;
    GRUCell decomposes cleanly).
    """

    def __init__(self):
        super().__init__()
        self.encoder = _PathCostmapEncoder()
        self.cond = nn.Linear(CTX_DIM + PATH_EMBED, _PATH_GRU_HID)
        self.seeds = nn.Parameter(torch.randn(PATH_K, _PATH_GRU_HID) * 0.1)
        self.cell = nn.GRUCell(PATH_DIM + _PATH_GRU_HID, _PATH_GRU_HID)
        self.head = nn.Linear(_PATH_GRU_HID, PATH_DIM)
        self.register_buffer('fan', torch.linspace(-_PATH_FAN_W, _PATH_FAN_W, PATH_K))

    def _conditioning(self, context, costmap):
        return self.cond(torch.cat([context, self.encoder(costmap)], dim=-1))

    def forward(self, context, costmap):
        cond = self._conditioning(context, costmap)
        b = cond.shape[0]
        outs = []
        for k in range(PATH_K):
            cond_k = cond + self.seeds[k].unsqueeze(0)
            h = cond_k
            prev = torch.zeros(b, PATH_DIM, device=cond.device, dtype=cond.dtype)
            steps = []
            for _ in range(PATH_H):
                h = self.cell(torch.cat([prev, cond_k], dim=-1), h)
                prev = self.head(h)
                steps.append(prev)
            outs.append(torch.stack(steps, dim=1).unsqueeze(1))
        return torch.cat(outs, dim=1)

    def recon_loss(self, context, costmap, target):
        """Regress the K candidates onto a lateral fan around the routing expert."""
        out = self(context, costmap)                        # [B, K, H, 2]
        tgt_x = target[..., 0].unsqueeze(1).expand(-1, PATH_K, -1)
        tgt_y = target[..., 1].unsqueeze(1) + self.fan.view(1, PATH_K, 1)
        tgt = torch.stack([tgt_x, tgt_y], dim=-1)           # [B, K, H, 2]
        return ((out - tgt) ** 2).mean()


def _gauss_kernel2d(sigma, device, dtype):
    """Build a normalized 2-D Gaussian conv kernel [1, 1, 2r+1, 2r+1]."""
    r = max(1, int(round(3.0 * sigma)))
    xs = torch.arange(-r, r + 1, device=device, dtype=dtype)
    g = torch.exp(-(xs ** 2) / (2.0 * sigma * sigma))
    g = g / g.sum()
    return torch.outer(g, g).view(1, 1, 2 * r + 1, 2 * r + 1), r


def _footprint_penalty(paths, costmap, inflate_cells=0, blur_sigma=0.0, interp=8):
    """
    Differentiable footprint-clearance penalty against the goal-aligned patch.

    Samples a (constant) obstacle-proximity field built from the costmap patch at
    points densely interpolated along each candidate and penalizes overlap, so
    training optimizes the proposals to be *what the deterministic validity layer
    accepts* — not just to imitate an expert. This is the "footprint enters the
    loss" lever flagged as future work in docs/generative_limits.md.

    Two refinements make the gradient actually route a centred path into an
    off-centre slot:

    * **Dense interpolation** (``interp`` points per segment) samples the wall
      *crossing* like the C++ ``isPathValid`` does, regardless of where the H
      waypoints happen to land — a raw per-waypoint penalty misses the crossing.
    * **Gaussian blur** (``blur_sigma`` cells) turns the binary occupancy into a
      smooth proximity field with a valley at the slot, so a waypoint stranded in
      the wall interior — where raw occupancy is flat and its gradient is zero —
      still feels a pull toward the free channel.

    ``paths`` is [B, K, H, 2] in the aligned frame (x forward, y lateral, metres);
    ``costmap`` is [B, 1, S, S] occupancy in [0, 1] (row -> forward x, col ->
    lateral y), matching the C++ ``alignedPatch`` resampling. ``inflate_cells``
    dilates obstacles by a max-pool first (the benchmark validator itself uses no
    inflation, so keep it small).
    """
    field = costmap
    if inflate_cells > 0:
        ksz = 2 * inflate_cells + 1
        field = nn.functional.max_pool2d(field, ksz, stride=1, padding=inflate_cells)
    if blur_sigma > 0.0:
        kern, r = _gauss_kernel2d(blur_sigma, field.device, field.dtype)
        field = nn.functional.conv2d(field, kern, padding=r)
    b, k, h, _ = paths.shape
    if interp > 1:
        a = paths[:, :, :-1, :].unsqueeze(3)                # [B,K,H-1,1,2]
        c = paths[:, :, 1:, :].unsqueeze(3)
        ts = torch.linspace(0.0, 1.0, interp, device=paths.device,
                            dtype=paths.dtype).view(1, 1, 1, interp, 1)
        pts = (a + (c - a) * ts).reshape(b, k, -1, 2)       # densified along H
    else:
        pts = paths
    x = pts[..., 0]
    y = pts[..., 1]
    # Aligned metres -> grid_sample normalized coords. grid[...,0]=width (cols ->
    # lateral y in [-half, half]); grid[...,1]=height (rows -> forward x in [0, fwd]).
    gx = (y / PATCH_HALF).clamp(-1.2, 1.2)
    gy = (2.0 * x / PATCH_FWD - 1.0).clamp(-1.2, 1.2)
    n = x.shape[-1]
    grid = torch.stack([gx, gy], dim=-1).reshape(b, k * n, 1, 2)
    occ = nn.functional.grid_sample(
        field, grid, mode='bilinear', align_corners=False, padding_mode='border')
    return (occ ** 2).mean()


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
    """
    Build a wall spanning the patch width at forward band [x_lo, x_hi] with one slot.

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


def make_costmap_path_centred_gap_dataset(num_samples):
    """
    Dead-ahead-gap data: a wall with a slot centred on the straight line.

    The mirror of the off-centre-gap data: the slot sits at lateral 0, so the
    expert goes *straight through* (no detour). Without these samples a model
    trained only on off-centre gaps over-aims and misses a gap that is dead ahead
    (the *centred gap* / *narrow gap* benchmark courses). Narrow slot half-widths
    are included so the model learns tight on-line passages too.
    """
    contexts = []
    patches = []
    targets = []
    spans = [(1.6, 2.4), (2.5, 3.3), (2.2, 3.0), (2.8, 3.6)]
    slot_hws = [0.3, 0.5, 0.4, 0.35]          # include narrow (0.3) on-line slots
    i = 0
    while len(contexts) < num_samples:
        d = 4.0 + 1.5 * ((i * 3) % 5) / 4.0   # goal distance 4.0..5.5 m
        x_lo, x_hi = spans[i % len(spans)]
        slot_hw = slot_hws[i % len(slot_hws)]
        contexts.append([d, 0.0])
        patches.append(_gap_patch(0.0, x_lo, x_hi, slot_hw))
        targets.append([[(h / (PATH_H - 1)) * d, 0.0] for h in range(PATH_H)])
        i += 1
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.stack(patches),
        torch.tensor(targets, dtype=torch.float32),
    )


def make_costmap_path_slalom_dataset(num_samples):
    """
    Slalom data: two staggered walls forcing an S-shaped (two-crossing) detour.

    The hardest gap shape: wall A has its slot to one side and wall B (further
    ahead) to the other, so the expert path must weave through slot A then slot B —
    an S, i.e. *two* lateral crossings. Single-bow proposers can't represent this;
    teaching the transformer the S directly (a two-Gaussian-bump expert over the
    aligned waypoints) is what lets a pure-generative model thread the *slalom*
    course. Both A-low/B-high and the mirror A-high/B-low are emitted.
    """
    contexts = []
    patches = []
    targets = []
    offs = [1.6, 2.0]                          # |lateral| slot offset [m]
    a_spans = [(0.9, 1.6), (1.2, 1.9)]         # wall A forward band [m]
    b_spans = [(2.4, 3.1), (2.7, 3.4)]         # wall B forward band [m]
    hws = [0.7, 0.85]                          # slot half-width [m]
    sigma = 0.18
    i = 0
    while len(contexts) < num_samples:
        d = 4.0 + 1.2 * ((i * 3) % 4) / 3.0    # goal distance 4.0..5.2 m
        off = offs[i % len(offs)]
        a_lo, a_hi = a_spans[(i // 2) % len(a_spans)]
        b_lo, b_hi = b_spans[(i // 2) % len(b_spans)]
        hw = hws[(i // 2) % len(hws)]
        t_a = min(0.9, max(0.1, 0.5 * (a_lo + a_hi) / d))
        t_b = min(0.95, max(0.15, 0.5 * (b_lo + b_hi) / d))
        for sign in (1.0, -1.0):               # A on -y then B on +y, and mirror
            if len(contexts) >= num_samples:
                break
            slot_a = -sign * off
            slot_b = sign * off
            rows = []
            for h in range(PATH_H):
                t = h / (PATH_H - 1)
                y = (slot_a * math.exp(-((t - t_a) / sigma) ** 2) +
                     slot_b * math.exp(-((t - t_b) / sigma) ** 2))
                rows.append([t * d, y])
            patch = torch.maximum(_gap_patch(slot_a, a_lo, a_hi, hw),
                                  _gap_patch(slot_b, b_lo, b_hi, hw))
            contexts.append([d, 0.0])
            patches.append(patch)
            targets.append(rows)
        i += 1
    return (
        torch.tensor(contexts, dtype=torch.float32),
        torch.stack(patches),
        torch.tensor(targets, dtype=torch.float32),
    )


def _path_dataset(dataset, num_samples):
    """Select/combine: 'side', 'gap', 'centred', 'slalom', or 'both'."""
    if dataset == 'side':
        return make_costmap_path_dataset(num_samples)
    if dataset == 'gap':
        return make_costmap_path_gap_dataset(num_samples)
    if dataset == 'centred':
        return make_costmap_path_centred_gap_dataset(num_samples)
    if dataset == 'slalom':
        return make_costmap_path_slalom_dataset(num_samples)
    if dataset == 'both':
        # Tri-mix: one-sided obstacles + off-centre gaps + dead-ahead (centred) gaps.
        # (Slalom was tried as a 4th component, but the transformer + lateral-fan
        # architecture can't thread it even trained slalom-only — a lateral fan
        # shifts both S-crossings off their slots; see make_costmap_path_slalom_dataset
        # and docs/generative_limits.md. Slalom stays the 'slalom' option for the
        # architecture work that would be needed, and out of the shipped 'both'.)
        third = num_samples // 3
        a = make_costmap_path_dataset(num_samples - 2 * third)
        b = make_costmap_path_gap_dataset(third)
        c = make_costmap_path_centred_gap_dataset(third)
        return tuple(torch.cat([a[i], b[i], c[i]], dim=0) for i in range(3))
    raise ValueError('unknown path dataset: ' + dataset)


def train_and_export_costmap_path(path, num_samples=96, epochs=400, lr=0.01, steps=4,
                                  kind='flow', dataset='side', device=None,
                                  footprint=0.0, inflate_cells=0, blur_sigma=0.0):
    """
    Train a costmap-conditioned Mode B path planner and export a 2-input ONNX.

    ``kind`` selects the family: ``'flow'`` (CostmapPathFlowPlanner),
    ``'transformer'`` (CostmapPathTransformerPlanner) or ``'recurrent'``
    (CostmapPathRecurrentPlanner). ``dataset`` selects the
    training data: ``'side'`` (one-sided obstacle, the default / shipped behaviour),
    ``'gap'`` (off-centre slot routing), or ``'both'``. ``device`` selects the
    training device (e.g. ``'cuda'``); the model is always exported on CPU so the
    artifact is portable.

    ``footprint`` (> 0) adds a differentiable footprint-clearance term
    (``_footprint_penalty``) on top of the fan-recon loss for the set-prediction
    families (``transformer`` / ``recurrent``), optimizing the proposals to thread
    the actual free channel of each patch — the validator-aware lever for the
    off-centre-gap ceiling. ``inflate_cells`` dilates obstacles in that term.
    """
    torch.manual_seed(0)
    dev = torch.device(device) if device is not None else torch.device('cpu')
    if kind == 'flow':
        model = CostmapPathFlowPlanner(steps=steps)
    elif kind == 'transformer':
        model = CostmapPathTransformerPlanner()
    elif kind == 'recurrent':
        model = CostmapPathRecurrentPlanner()
    else:
        raise ValueError('unknown path planner kind: ' + kind)
    if footprint > 0.0 and kind == 'flow':
        raise ValueError('footprint loss requires a fan set-prediction kind '
                         '(transformer / recurrent), not flow')
    model.to(dev)
    context, costmap, target = _path_dataset(dataset, num_samples)
    context, costmap, target = context.to(dev), costmap.to(dev), target.to(dev)
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    model.train()
    loss = torch.tensor(0.0)
    for _ in range(max(1, epochs)):
        optimizer.zero_grad()
        if footprint > 0.0:
            # Validator-aware training: one full-batch forward feeds fan-recon,
            # smoothness and the footprint-clearance penalty together.
            out = model(context, costmap)                   # [B, K, H, 2]
            tgt_x = target[..., 0].unsqueeze(1).expand(-1, PATH_K, -1)
            tgt_y = target[..., 1].unsqueeze(1) + model.fan.view(1, PATH_K, 1)
            tgt = torch.stack([tgt_x, tgt_y], dim=-1)
            loss = ((out - tgt) ** 2).mean()
            loss = loss + footprint * _footprint_penalty(
                out, costmap, inflate_cells, blur_sigma)
        elif kind == 'flow':
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
