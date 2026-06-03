"""Render a costmap-conditioned avoidance demo from the real CostmapFlowPlanner.

Trains the shipped ``CostmapFlowPlanner`` (flow matching + egocentric costmap
encoder) on a synthetic obstacle-sweep dataset and renders its K candidate
trajectories as the obstacle moves left<->right, producing ``docs/costmap_demo.gif``.

Reproduce::

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash          # so nav2_diffusion_training is importable
    pip install torch imageio matplotlib
    python3 tools/costmap_demo.py       # writes /tmp/gifdata/costmap_demo.gif

This is a visualization aid (synthetic data, CPU); it is not a benchmark.
"""
import math

import imageio.v2 as imageio
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402

from nav2_diffusion_training.generative_planners import (  # noqa: E402
    CostmapFlowPlanner, COSTMAP_SIZE, HORIZON)

S = COSTMAP_SIZE
RES = 0.03
W = 8  # obstacle block width [cells]


def obstacle_patch(col0):
    """A patch with a vertical obstacle block starting at column col0."""
    patch = torch.zeros(1, 1, S, S)
    patch[:, :, S // 3:2 * S // 3, col0:col0 + W] = 1.0
    return patch


def make_dataset(n):
    """Continuous obstacle position -> expert veers away (lateral, proportional)."""
    ctx, cmaps, tgt = [], [], []
    for i in range(n):
        frac = (i % 9) / 8.0
        col0 = int(4 + frac * (S - W - 8))
        center = col0 + W / 2.0
        lat_dir = 1.0 if center > S / 2 else -1.0  # +y (left in base frame) = away
        # closeness: 1 when obstacle is centered ahead, ->0 at the edges
        close = max(0.0, 1.0 - abs(center - S / 2) / (S / 2))
        rows = []
        for h in range(HORIZON):
            f = (h + 1) / HORIZON
            fwd = 0.66 * f                              # reach across the frame
            bump = math.sin(math.pi * f)               # smooth out-and-back arc
            lat = lat_dir * (0.10 + 0.22 * close) * bump
            yaw = lat_dir * (0.10 + 0.22 * close) * math.cos(math.pi * f) * 0.5
            rows.append([fwd, lat, yaw])
        ctx.append([1.0, 0.0, 0.3, 1.0])
        cmaps.append(obstacle_patch(col0)[0])
        tgt.append(rows)
    return (torch.tensor(ctx), torch.stack(cmaps), torch.tensor(tgt))


def train():
    """Train the shipped CostmapFlowPlanner on the sweep dataset."""
    torch.manual_seed(0)
    model = CostmapFlowPlanner(steps=8)
    ctx, cm, tgt = make_dataset(72)
    opt = torch.optim.Adam(model.parameters(), lr=0.01)
    for _ in range(800):
        opt.zero_grad()
        loss = model.flow_loss(ctx, cm, tgt)
        out = model(ctx[:16], cm[:16])               # [B, K, H, 3]
        jerk = out[:, :, 2:, :] - 2 * out[:, :, 1:-1, :] + out[:, :, :-2, :]
        loss = loss + 5.0 * (jerk ** 2).mean()        # smoothness regularizer
        loss.backward()
        opt.step()
    model.eval()
    with torch.no_grad():
        model.latents.mul_(0.22)
    return model


def to_cells(fx, fy):
    """Base-frame (forward, left) -> patch (col, row)."""
    center = (S - 1) / 2.0
    return center - fy / RES, center - fx / RES


def render(model):
    """Sweep the obstacle left<->right and draw the model's candidates."""
    cols = list(range(4, S - W - 4, 1))
    sweep = cols + cols[::-1]
    images = []
    ctx = torch.tensor([[1.0, 0.0, 0.3, 1.0]])
    for col0 in sweep:
        patch = obstacle_patch(col0)
        with torch.no_grad():
            cands = model(ctx, patch)[0].numpy()  # [K, H, 3]
        fig, ax = plt.subplots(figsize=(5, 5), dpi=80)
        ax.imshow(patch[0, 0].numpy(), cmap='Reds', vmin=0, vmax=1, alpha=0.85)
        for k in range(cands.shape[0]):
            xs, ys = [], []
            for h in range(cands.shape[1]):
                c, r = to_cells(float(cands[k, h, 0]), float(cands[k, h, 1]))
                xs.append(c)
                ys.append(r)
            ax.plot(xs, ys, color='#3fb950' if k == 0 else '#2f81f7', lw=2.4 if k == 0 else 1.4)
        ax.plot((S - 1) / 2.0, (S - 1) / 2.0, marker='o', color='#e6edf3', markersize=8)
        ax.set_xlim(0, S - 1)
        ax.set_ylim(S - 1, 0)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title('costmap-conditioned flow: candidates veer away from the obstacle',
                     fontsize=8)
        fig.tight_layout(pad=0.3)
        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = np.asarray(fig.canvas.buffer_rgba()).reshape(h, w, 4)[:, :, :3].copy()
        images.append(img)
        plt.close(fig)
    imageio.mimsave('/tmp/gifdata/costmap_demo.gif', images, duration=0.08, loop=0)
    print('wrote /tmp/gifdata/costmap_demo.gif frames=%d' % len(images))


if __name__ == '__main__':
    render(train())
