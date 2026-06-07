# Model Card: diffusion_local_costmap_threading_v0

> The first learned **Mode A** (local controller) model in this repo to **thread an
> obstacle purely generatively** (no classical fallback) in the real C++
> `controller_benchmark`. It is the costmap-token transformer **DAgger-trained on a
> corrected reactive dodge oracle**, run with the controller's **windowed footprint gate**
> (`safety_check_points`). Manifest: [manifest.yaml](manifest.yaml). Reproduce:
> [export.py](export.py).

## Summary

- **Model family:** transformer set-prediction (same net as the transformer Mode A model),
  but **DAgger closed-loop trained** on a corrected oracle.
- **Task:** obstacle-threading local trajectory proposal for Nav2 **Mode A**.
- **Inputs:** egocentric costmap patch (32×32) + **[goal_x, goal_y, linear_speed, max_angular_speed]**.
- **Output:** 3 candidate trajectories × 10 SE(2) waypoints (base frame).
- **Artifact:** `costmap_threading.onnx` (reproducible from `export.py`).

## What it shows — cracking the Mode A obstacle-threading ceiling

The documented Mode A ceiling (the learned controller stalls in front of obstacles) was
diagnosed (docs/generative_limits.md) to **two concrete mechanisms — not model capacity**:

1. **The shipped DAgger oracle itself collided** — a 0.20 m transient bow plus an on-line
   carrot drives pure-pursuit straight into the block (expert-only closed-loop 1/4), so
   DAgger aggregated colliding labels. The **corrected** oracle commits a *sustained*
   lateral offset to the free side via a curvature-based dodge (pure-pursuit toward a
   close, offset carrot) and clears the block.
2. **The deployed full-horizon footprint gate hard-rejects a tight reactive skirt** whose
   1 m lookahead clips the block, though step-wise execution skirts it safely. The
   controller now supports a **windowed gate** (`safety_check_points`): validate only the
   leading points the robot executes before re-planning (receding-horizon; the live
   costmap is re-checked every cycle).

With both, the DAgger-trained **transformer** reaches the goal **4/5 closed-loop** in the
costmap sim (the small CNN-embedding flow model cannot fit the sharp dodge — it stays at
1/4; capacity matters).

**Result (real C++ `controller_benchmark`, `safety_check_points=3`, no fallback):**

| scenario | learned / transformer / recurrent | **threading** |
|---|:-:|:-:|
| open | reached | reached |
| **side obstacle** | timeout (~1.0 m) | **reached (4.27 m traverse)** |
| frontal obstacle (dead-ahead) | timeout (~1.0 m, 0.75 m) | timeout (1.69 m, **0.20 m** clearance) |
| corridor | timeout | timeout |

It is the **first learned Mode A model here to thread an obstacle generatively** (the side
obstacle). On the dead-ahead *frontal* block it drives markedly further and closer but
does not complete; the *corridor* needs centring, which the dodge oracle does not do.

## Honest scope — what it does NOT do

- **Not a full solve.** Threads the *side obstacle* generatively, but the dead-ahead
  inflated *frontal* block and the *corridor* still time out — the sim 4/5 does not fully
  transfer (the live costmap inflates the dead-ahead block; the corridor is out of the
  dodge oracle's design). The **hybrid (VFH+ fallback)** remains the all-scenario guarantee.
- **Requires** the windowed footprint gate (`safety_check_points>0`). Under the default
  full-horizon gate it is hard-rejected like the other learned models.
- **Synthetic data only.** Never validated on a real robot or rosbag.
- **Research / demonstration model.** Do not deploy on hardware; the safety layer is the authority.

## Reproducibility

- **Command:** `PYTHONPATH=../../generative/nav2_diffusion_training python3 export.py`
- **Seed:** 0 · **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA, exported on CPU.
- **Hyperparameters:** `CostmapTransformerPlanner`; DAgger iters 8, base 320, epochs 900,
  lr 0.003 cosine, grad-clip 1.0, best-checkpoint; `SAFETY_WINDOW` 3.
- DAgger + GPU training is not bit-exact run-to-run; the checksum is fixed in the manifest
  and a re-export reproduces the threading behaviour.

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated) · Code: Apache-2.0
