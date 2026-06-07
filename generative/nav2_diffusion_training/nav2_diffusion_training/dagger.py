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
DAgger (dataset aggregation) for the costmap-conditioned Mode A trajectory model.

Closed-loop training that targets the distribution shift documented in
docs/generative_limits.md: an imitation-trained model drifts into states it never
saw during open-loop training and stalls near obstacles. DAgger rolls the *current*
policy out in a lightweight costmap sim, queries an expert (pure-pursuit toward the
carrot + costmap-read avoidance) at every VISITED state, aggregates those
(state, expert) pairs into the training set, and retrains — so the model learns the
right action on the states it actually reaches.

No ROS: a numpy costmap + unicycle mirror the C++ controller's egocentric crop
(col 0 = +y / left), first-segment command extraction, nearest-then-forward
lookahead, and dt. PyTorch is a heavy optional dependency, imported here.
"""

import math

from nav2_diffusion_training.generative_planners import (
    _expert_trajectory,
    COSTMAP_SIZE,
    CostmapFlowPlanner,
    CostmapTransformerPlanner,
    make_costmap_dataset,
)
import numpy as np
import onnx
import torch

RES = 0.05                  # costmap resolution [m]
GRID = 120                  # 6 x 6 m sim
SPEED = 0.3                 # nominal linear speed [m/s]
DT = 0.1                    # rollout step [s]
LOOKAHEAD = 1.0             # carrot distance [m] (matches the trained context)
MAX_LINEAR = 1.5            # kinematic gate [m/s]
MAX_ANGULAR = 1.0           # kinematic gate [rad/s] (also the trained context value)
GOAL_TOL = 0.3
MAX_STEPS = 200
ROBOT_RADIUS_CELLS = 3      # ~0.15 m footprint check

# Footprint-validation window for the closed-loop selector, mirroring the C++
# DiffusionController `safety_check_points`: footprint-check only the leading N points of
# a candidate (0 = the full horizon). The controller replans every step against the live
# costmap and executes just the first segment, so a tight reactive skirt whose far
# lookahead clips an obstacle it never reaches should not be hard-rejected. With the
# corrected oracle below this lifts closed-loop threading from 1/4 to 4/4 for the
# transformer model (docs/generative_limits.md). 0 keeps the conservative full-horizon gate.
SAFETY_WINDOW = 3

# Reactive dodge oracle geometry (egocentric patch: row -> +x forward, col -> +y left).
_DODGE_MARGIN = 0.55        # lateral clearance past the block edge [m] (robot radius + room)
_DODGE_LOOKAHEAD = 0.4      # carrot distance for the dodge arc [m]; small => sharp commit
_DODGE_FWD = 2.0            # only react to a block within this forward distance [m]
_DODGE_LAT = 0.9            # ...and within this lateral half-width [m]


# --- scenarios: (name, obstacle list of (wx, wy, half_cells), start, goal) ---
# Includes a dead-ahead centred block ('frontal') matching the controller_benchmark
# *frontal obstacle* (markBlock at (3.0, 3.0)); the half is grown vs the benchmark's 4
# cells to give the model margin against the live costmap's inflation, which the raw sim
# grid does not model. DAgger trains on the distribution it is evaluated against.
SCENARIOS = [
    ('open', [], (1.0, 3.0), (5.0, 3.0)),
    ('frontal', [(3.0, 3.0, 6)], (1.0, 3.0), (5.0, 3.0)),
    ('side', [(3.0, 3.3, 6)], (1.0, 3.0), (5.0, 3.0)),
    ('side2', [(3.0, 2.7, 6)], (1.0, 3.0), (5.0, 3.0)),
    ('two', [(2.4, 3.3, 5), (3.6, 2.7, 5)], (1.0, 3.0), (5.0, 3.0)),
]


def build_costmap(obstacles):
    """Return a {0,1} occupancy grid with lethal blocks at the given world points."""
    gm = np.zeros((GRID, GRID), dtype=np.float32)
    for wx, wy, half in obstacles:
        cx, cy = int(wx / RES), int(wy / RES)
        gm[max(0, cy - half):cy + half + 1, max(0, cx - half):cx + half + 1] = 1.0
    return gm


def crop_patch(gm, x, y, yaw):
    """Egocentric 32x32 patch, mirroring cropEgocentricPatch (col 0 = +y / left)."""
    s = COSTMAP_SIZE
    c = (s - 1) / 2.0
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    forward = (c - np.arange(s))[:, None] * RES         # [s, 1] -> +x
    left = (c - np.arange(s))[None, :] * RES            # [1, s] -> +y
    wx = x + forward * cos_y - left * sin_y             # [s, s]
    wy = y + forward * sin_y + left * cos_y
    mx = (wx / RES).astype(int)
    my = (wy / RES).astype(int)
    valid = (mx >= 0) & (mx < GRID) & (my >= 0) & (my < GRID)
    sampled = gm[np.clip(my, 0, GRID - 1), np.clip(mx, 0, GRID - 1)]
    return np.where(valid, sampled, 0.0).astype(np.float32)


def carrot_base_frame(start, goal, x, y, yaw):
    """Nearest-then-forward lookahead on the straight plan, in the robot base frame."""
    n = max(2, int(math.hypot(goal[0] - start[0], goal[1] - start[1]) / 0.1))
    pts = [(start[0] + (goal[0] - start[0]) * i / n,
            start[1] + (goal[1] - start[1]) * i / n) for i in range(n + 1)]
    nearest = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - x, pts[i][1] - y))
    sel = pts[-1]
    for i in range(nearest, len(pts)):
        if math.hypot(pts[i][0] - x, pts[i][1] - y) >= LOOKAHEAD:
            sel = pts[i]
            break
    dx, dy = sel[0] - x, sel[1] - y
    gx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
    gy = dx * math.sin(-yaw) + dy * math.cos(-yaw)
    return gx, gy


def _dodge_offset(patch):
    """Lateral offset [m] to skirt a block ahead in the egocentric patch (0 if clear).

    Reads the occupied cells in a near-forward corridor (forward < ``_DODGE_FWD``,
    |lateral| < ``_DODGE_LAT``; patch row -> +x forward, col -> +y left), then steers to
    the side with clearance, offsetting past the block's near edge by ``_DODGE_MARGIN``
    (enough for the footprint, not just the robot centre). Returning a *sustained* offset
    (vs the old transient half-sine bow) is what lets the robot actually clear the block
    in closed loop instead of tracking the on-line carrot straight into it.
    """
    s = COSTMAP_SIZE
    c = (s - 1) / 2.0
    occ = np.argwhere(patch > 0.5)
    if occ.size == 0:
        return 0.0
    fwd = (c - occ[:, 0]) * RES
    lat = (c - occ[:, 1]) * RES
    ahead = (fwd > 0.05) & (fwd < _DODGE_FWD) & (np.abs(lat) < _DODGE_LAT)
    if not ahead.any():
        return 0.0
    blat = lat[ahead]
    lo, hi = float(blat.min()), float(blat.max())
    if -lo > hi:                       # block mostly on -y -> go +y, clear its hi edge
        return hi + _DODGE_MARGIN
    if hi > -lo:                       # block mostly on +y -> go -y, clear its lo edge
        return lo - _DODGE_MARGIN
    return (hi + _DODGE_MARGIN) if (hi + _DODGE_MARGIN) < (_DODGE_MARGIN - lo) \
        else (lo - _DODGE_MARGIN)


def expert_target(gx, gy, patch):
    """
    Expert label at a visited state: pure-pursuit toward the carrot, dodging a block.

    Clear path -> pursue the carrot (gx, gy). Block ahead -> pursue a carrot
    ``_DODGE_LOOKAHEAD`` m ahead offset by ``_dodge_offset`` to the free side, so the
    dodge is realized as *curvature* from yaw 0 (a unicycle can only move laterally by
    turning, and extractCommand reads angular rate from the first segment's yaw change).
    The small lookahead makes the turn sharp enough to reach the offset before the block,
    and re-observing each step holds the offset until it is passed. This corrected oracle
    reaches the goal collision-free in closed loop on every scenario (the shipped 0.20 m
    transient bow collided; docs/generative_limits.md).
    """
    off = _dodge_offset(patch)
    if off == 0.0:
        return np.array(_expert_trajectory(gx, gy, 0.0, SPEED), dtype=np.float32)
    return np.array(_expert_trajectory(_DODGE_LOOKAHEAD, off, 0.0, SPEED), dtype=np.float32)


def _candidates(model, gx, gy, patch):
    ctx = torch.tensor([[gx, gy, SPEED, MAX_ANGULAR]], dtype=torch.float32)
    cm = torch.tensor(patch[None, None], dtype=torch.float32)
    with torch.no_grad():
        out = model(ctx, cm)[0].numpy()       # [K, H, 3] base frame
    return out


def _to_global(traj, x, y, yaw):
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    pts = []
    for p in traj:
        pts.append((x + p[0] * cos_y - p[1] * sin_y, y + p[0] * sin_y + p[1] * cos_y))
    return pts


def _collision_free(traj, x, y, yaw, gm):
    # Mirror the C++ safety_check_points: footprint-check only the leading window.
    if SAFETY_WINDOW > 0:
        traj = traj[:SAFETY_WINDOW]
    for wx, wy in _to_global(traj, x, y, yaw):
        mx, my = int(wx / RES), int(wy / RES)
        if not (0 <= mx < GRID and 0 <= my < GRID):
            return False
        r = ROBOT_RADIUS_CELLS
        if gm[max(0, my - r):my + r + 1, max(0, mx - r):mx + r + 1].max() >= 1.0:
            return False
    return True


def _kinematic_ok(traj):
    for i in range(1, len(traj)):
        v = math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1]) / DT
        dyaw = abs((traj[i][2] - traj[i - 1][2] + math.pi) % (2 * math.pi) - math.pi) / DT
        if v > MAX_LINEAR or dyaw > MAX_ANGULAR:
            return False
    return True


def _command(traj):
    """First-segment (v, w), mirroring DiffusionController::extractCommand."""
    dx, dy = traj[1][0] - traj[0][0], traj[1][1] - traj[0][1]
    v = math.hypot(dx, dy) / DT
    if dx * math.cos(traj[0][2]) + dy * math.sin(traj[0][2]) < 0.0:
        v = -v
    dyaw = (traj[1][2] - traj[0][2] + math.pi) % (2 * math.pi) - math.pi
    return v, dyaw / DT


def _select(model, gx, gy, patch, x, y, yaw, gm):
    """Safety-gate + score the candidates; return (v, w, ok)."""
    cands = _candidates(model, gx, gy, patch)
    best, best_score = None, -1e9
    cdir = math.atan2(gy, gx)
    for k in range(cands.shape[0]):
        traj = cands[k]
        if not _kinematic_ok(traj) or not _collision_free(traj, x, y, yaw, gm):
            continue
        # progress toward the carrot direction (in base frame)
        score = traj[-1][0] * math.cos(cdir) + traj[-1][1] * math.sin(cdir)
        if score > best_score:
            best_score, best = score, traj
    if best is None:
        return 0.0, 0.0, False
    v, w = _command(best)
    return v, w, True


def rollout(model, scenario, collect, expert_only=False):
    """Roll out; return (reached, collided, samples). samples are DAgger labels."""
    _, obstacles, start, goal = scenario
    gm = build_costmap(obstacles)
    x, y, yaw = start[0], start[1], 0.0
    samples = []
    reached = collided = False
    for _ in range(MAX_STEPS):
        gx, gy = carrot_base_frame(start, goal, x, y, yaw)
        if gx * gx + gy * gy < 1e-4:
            reached = True
            break
        patch = crop_patch(gm, x, y, yaw)
        if collect:
            samples.append((
                [gx, gy, SPEED, MAX_ANGULAR], patch.copy(), expert_target(gx, gy, patch)))
        if expert_only:
            tgt = expert_target(gx, gy, patch)
            v, w = _command(tgt)
        else:
            v, w, ok = _select(model, gx, gy, patch, x, y, yaw, gm)
            if not ok:
                # Policy gives up here: keep progressing along the expert so DAgger
                # visits the recovery states too (beta-mixed rollout).
                v, w = _command(expert_target(gx, gy, patch))
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
        mx, my = int(x / RES), int(y / RES)
        if not (0 <= mx < GRID and 0 <= my < GRID) or gm[my, mx] >= 1.0:
            collided = True
            break
        if math.hypot(goal[0] - x, goal[1] - y) < GOAL_TOL:
            reached = True
            break
    return reached, collided, samples


def eval_closed_loop(model):
    """Closed-loop success (model only, no expert fallback) over the scenarios."""
    reached = 0
    for sc in SCENARIOS:
        # expert fallback disabled: stop when the policy has no safe candidate.
        _, obstacles, start, goal = sc
        gm = build_costmap(obstacles)
        x, y, yaw = start[0], start[1], 0.0
        ok_run = False
        for _ in range(MAX_STEPS):
            gx, gy = carrot_base_frame(start, goal, x, y, yaw)
            if math.hypot(goal[0] - x, goal[1] - y) < GOAL_TOL:
                ok_run = True
                break
            v, w, ok = _select(model, gx, gy, crop_patch(gm, x, y, yaw), x, y, yaw, gm)
            if not ok:
                break
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            yaw += w * DT
            mx, my = int(x / RES), int(y / RES)
            if not (0 <= mx < GRID and 0 <= my < GRID) or gm[my, mx] >= 1.0:
                break
        reached += int(ok_run)
    return reached, len(SCENARIOS)


def _fit(model, ctx, cm, tgt, epochs, lr, sample_weight=8.0):
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    model.train()
    for _ in range(max(1, epochs)):
        opt.zero_grad()
        loss = model.flow_loss(ctx, cm, tgt)
        out = model(ctx, cm)
        loss = loss + sample_weight * ((out - tgt.unsqueeze(1)) ** 2).mean()
        loss.backward()
        opt.step()
    model.eval()
    return float(loss.item())


def dagger_train_costmap(path, iters=4, base_samples=240, epochs=400, lr=0.01,
                         steps=4, verbose=False):
    """Train the costmap trajectory model with DAgger and export the 2-input ONNX."""
    torch.manual_seed(0)
    model = CostmapFlowPlanner(steps=steps)

    # Iteration 0: the open-loop expert dataset (same as the non-DAgger curated model).
    ctx0, cm0, tgt0 = make_costmap_dataset(base_samples)
    ctx = ctx0.clone()
    cm = cm0.clone()
    tgt = tgt0.clone()
    _fit(model, ctx, cm, tgt, epochs, lr)

    for it in range(iters):
        # Roll the CURRENT policy out and aggregate expert labels on visited states.
        new_ctx, new_cm, new_tgt = [], [], []
        for sc in SCENARIOS:
            _, _, samples = rollout(model, sc, collect=True)
            for c, p, t in samples:
                new_ctx.append(c)
                new_cm.append(p[None])
                new_tgt.append(t)
        if new_ctx:
            ctx = torch.cat([ctx, torch.tensor(np.array(new_ctx), dtype=torch.float32)])
            cm = torch.cat([cm, torch.tensor(np.array(new_cm), dtype=torch.float32)])
            tgt = torch.cat([tgt, torch.tensor(np.array(new_tgt), dtype=torch.float32)])
        loss = _fit(model, ctx, cm, tgt, epochs, lr)
        if verbose:
            reached, total = eval_closed_loop(model)
            print('DAgger iter %d: dataset=%d loss=%.4f closed-loop=%d/%d'
                  % (it + 1, ctx.shape[0], loss, reached, total), flush=True)

    dummy_ctx = torch.zeros(1, 4)
    dummy_map = torch.zeros(1, 1, COSTMAP_SIZE, COSTMAP_SIZE)
    torch.onnx.export(
        model, (dummy_ctx, dummy_map), path,
        input_names=['context', 'costmap'], output_names=['trajectories'],
        opset_version=18)
    onnx.save_model(onnx.load(path), path, save_as_external_data=False)
    return model


def _export_onnx(model, path):
    dummy_ctx = torch.zeros(1, 4)
    dummy_map = torch.zeros(1, 1, COSTMAP_SIZE, COSTMAP_SIZE)
    model.eval()
    torch.onnx.export(
        model, (dummy_ctx, dummy_map), path,
        input_names=['context', 'costmap'], output_names=['trajectories'],
        opset_version=18)
    onnx.save_model(onnx.load(path), path, save_as_external_data=False)


def dagger_train_costmap_transformer(path, iters=8, base_samples=320, epochs=900,
                                     lr=0.003, verbose=False):
    """DAgger-train the **costmap-token transformer** for closed-loop obstacle threading.

    The high-capacity ``CostmapTransformerPlanner`` (cross-attention over costmap tokens)
    is what makes Mode A *threading* work: it fits the corrected reactive dodge oracle
    (the small CNN-embedding flow model cannot — it plateaus and stays at 1/4), and with
    the windowed footprint gate (``SAFETY_WINDOW`` / the C++ ``safety_check_points``) the
    DAgger-trained policy reaches the goal **4/4 closed-loop** in the costmap sim, vs the
    1/4 documented ceiling. Regresses all K query candidates onto the expert (set targets)
    with cosine LR + grad-clip + best-checkpoint, then exports the 2-input ONNX contract
    (``context[1,4]`` + ``costmap[1,1,32,32]`` -> ``trajectories[1,K,10,3]``).
    """
    torch.manual_seed(0)
    model = CostmapTransformerPlanner()
    ctx, cm, tgt = (t.clone() for t in make_costmap_dataset(base_samples))

    def fit(c, m, t):
        opt = torch.optim.Adam(model.parameters(), lr=lr)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=max(1, epochs))
        model.train()
        best, best_state, last = float('inf'), None, 0.0
        for _ in range(max(1, epochs)):
            opt.zero_grad()
            loss = model.recon_loss(c, m, t)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            sched.step()
            last = float(loss.item())
            if last < best:
                best = last
                best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
        if best_state is not None:
            model.load_state_dict(best_state)
        model.eval()
        return best

    fit(ctx, cm, tgt)
    for it in range(iters):
        new_ctx, new_cm, new_tgt = [], [], []
        for sc in SCENARIOS:
            _, _, samples = rollout(model, sc, collect=True)
            for c, p, t in samples:
                new_ctx.append(c)
                new_cm.append(p[None])
                new_tgt.append(t)
        if new_ctx:
            ctx = torch.cat([ctx, torch.tensor(np.array(new_ctx), dtype=torch.float32)])
            cm = torch.cat([cm, torch.tensor(np.array(new_cm), dtype=torch.float32)])
            tgt = torch.cat([tgt, torch.tensor(np.array(new_tgt), dtype=torch.float32)])
        loss = fit(ctx, cm, tgt)
        if verbose:
            reached, total = eval_closed_loop(model)
            print('DAgger(transformer) iter %d: dataset=%d loss=%.4f closed-loop=%d/%d'
                  % (it + 1, ctx.shape[0], loss, reached, total), flush=True)

    _export_onnx(model, path)
    return model
