# Model Card: diffusion_local_costmap_recurrent_v0

> The curated generative **local controller** (Mode A) model in the **recurrent**
> family — a GRU autoregressive rollout planner, sibling to the flow and transformer
> Mode A models in [../diffusion_local](../diffusion_local/model_card.md) and
> [../diffusion_local_transformer](../diffusion_local_transformer/model_card.md).
> Manifest: [manifest.yaml](manifest.yaml). Reproduce: [export.py](export.py).

## Summary

- **Model family:** recurrent (GRU) autoregressive rollout
- **Task:** local **trajectory** proposal for the Nav2 **Mode A** controller
  (`nav2_diffusion_controller::DiffusionController` →
  `nav2_diffusion_onnx::OnnxTrajectoryModel`)
- **Robot kinematics:** differential drive (unicycle)
- **Inputs:** egocentric local-costmap patch (32×32) + carrot (base frame) + speed limits
- **Output:** 3 candidate base-frame SE(2) trajectories × 10 steps (~1 s), gated and
  scored by the controller's deterministic safety layer
- **Runtime / precision:** ONNX / fp32
- **Artifact:** `costmap_recurrent.onnx` (≈592 KB), checked in directly (small,
  reproducible from `export.py`).

## How it differs from the flow / transformer Mode A models

All three models target the identical ONNX contract and the same controller, so they
are drop-in interchangeable. The difference is the proposer architecture and, with
it, the inductive bias:

- **flow** (`../diffusion_local`): conditional flow matching, integrates an ODE over
  a few Euler steps from sampled noise; the K candidates come from K fixed latents.
- **transformer** (`../diffusion_local_transformer`): K learned query tokens
  cross-attend to a tokenized costmap patch + context token and each decodes a whole
  trajectory in **one forward pass** — *set prediction*.
- **recurrent** (this model): a CNN encodes the costmap patch to a conditioning
  vector and a **GRU emits the trajectory one point at a time**, feeding the previous
  point back in (*autoregressive rollout* — the world-model-style sequential bias).
  K=3 learned seed vectors give each candidate a distinct conditioning, so they
  differ without sampling noise. Its loss is a direct MSE onto the smooth
  pure-pursuit-arc expert, so the rollout is smooth and within the kinematic gate by
  construction (no `sample_weight` / integration-steps tuning). The `HORIZON`/`K`
  loops unroll into a static graph, so the GRU exports cleanly to ONNX.

## Intended use

Demonstration / research. Like its siblings it proves the *propose → safety-gate →
score → cmd_vel* controller architecture with a real learned proposer that reads the
egocentric costmap and biases **every** candidate away from a one-sided obstacle. It
is the fifth generative family in the model zoo (flow / diffusion / consistency /
transformer / recurrent), and the one whose generation is sequential rather than
one-shot or denoising.

## Out-of-scope / limitations

Honest and load-bearing (same competence envelope as the flow / transformer siblings):

- **Synthetic data only.** Never validated on a real robot or rosbag.
- **One-sided obstacles only.** A symmetric head-on block is ambiguous (never seen);
  breaking that symmetry is a reactive-controller job (VFH+/ND).
- **Short, gentle.** ~1 s / ~0.3 m horizon with ~0.13 m lateral veer; it leans on
  closed-loop replanning rather than one big swerve.
- **Autoregressive drift.** The rollout feeds its own previous point back in, so a
  single off step can compound; the kinematic + footprint gate, not the model, is the
  authority that catches it.
- **Carrot distribution.** Trained on carrots ~0.9–1.1 m ahead, bearing ±0.4 rad;
  far-off-axis carrots are out of distribution.
- **Small research model.** Expected to reach the goal closed-loop in the *open*
  scenario; on obstacle scenarios (out of training distribution) the safety layer
  stops it safely short rather than threading. Threading needs better data / capacity
  (see [../../docs/generative_limits.md](../../docs/generative_limits.md) and
  [../../docs/next_phase.md](../../docs/next_phase.md)).
- **No safety authority.** The kinematic + footprint filters gate every proposal; if
  none is safe the controller falls back or stops. Do not deploy on hardware.

## Training data

`nav2_diffusion_training.generative_planners.make_costmap_dataset` (240 samples) —
identical to the flow / transformer Mode A models: one-sided obstacle band on +y (low
cols, matching `cropEgocentricPatch`) or −y of the 32×32 patch with varied row-band /
column width, emitted as mirrored +y/−y pairs (no lateral bias) plus clear samples;
the carrot is varied in distance (~0.9–1.1 m) and bearing (±0.4 rad); the expert
(`_expert_trajectory`) follows a pure-pursuit arc toward the carrot with a half-sine
bow away from the obstacle and path-tangent yaw. Fully procedural; no real data.

## Benchmark results

Research placeholder → checked **behaviourally**, not against the full Nav2
deployment benchmark:

- **Exported-model behaviour (onnxruntime):** obstacle on +y/left → all 3 candidates
  veer −y (mean lateral −0.13 m); obstacle on −y/right → all veer +y (+0.13 m); clear
  → centred (~0); the autoregressive `x` increases monotonically to ~0.30 m over the
  horizon (= 0.3 m/s × 1 s, within the kinematic gate); finite, smooth.
- **Offline leaderboard:** included as `costmap-recurrent` and `recurrent` in
  [../../docs/model_comparison.md](../../docs/model_comparison.md) (regenerated with
  `tools/benchmark_models.py`).
- **End-to-end C++ side-selection:** asserted in `nav2_diffusion_onnx`'s
  `test_onnx_trajectory_model` (`CuratedZooRecurrentVeersAwayFromObstacle`, runs where
  onnxruntime is available).
- **Collisions:** gated by the deterministic kinematic + footprint safety layer (0).

## Safety

- The model has **no** safety authority. Every proposal passes the kinematic and
  footprint filters; if none is safe the controller delegates to its
  `fallback_controller_plugin` or stops — never an unsafe command.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../generative/nav2_diffusion_training python3 export.py`
  (uses CUDA when available; `CUDA_VISIBLE_DEVICES= ` forces a deterministic CPU build)
- **Seed:** 0 (`torch.manual_seed(0)` inside the training function)
- **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA, exported on CPU
- **Hyperparameters:** recurrent, 240 samples, 1500 epochs, lr 0.01 (the `recon_loss`
  is already a direct MSE onto the smooth expert, so no `sample_weight` or flow-step
  tuning is needed)
- Bit-for-bit reproduction may vary across torch versions / hardware (and CPU vs
  GPU); behaviour (side-selection, veer magnitude, forward rollout) is stable.

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
