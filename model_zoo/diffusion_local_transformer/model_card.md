# Model Card: diffusion_local_costmap_transformer_v0

> The curated generative **local controller** (Mode A) model in the **transformer**
> family — a DETR-style set-prediction planner, sibling to the flow Mode A model in
> [../diffusion_local](../diffusion_local/model_card.md). Manifest:
> [manifest.yaml](manifest.yaml). Reproduce: [export.py](export.py).

## Summary

- **Model family:** transformer set prediction (1 deterministic forward pass)
- **Task:** local **trajectory** proposal for the Nav2 **Mode A** controller
  (`nav2_diffusion_controller::DiffusionController` →
  `nav2_diffusion_onnx::OnnxTrajectoryModel`)
- **Robot kinematics:** differential drive (unicycle)
- **Inputs:** egocentric local-costmap patch (32×32) + carrot (base frame) + speed limits
- **Output:** 3 candidate base-frame SE(2) trajectories × 10 steps (~1 s), gated and
  scored by the controller's deterministic safety layer
- **Runtime / precision:** ONNX / fp32
- **Artifact:** `costmap_transformer.onnx` (≈224 KB), checked in directly (small,
  reproducible from `export.py`).

## How it differs from the flow Mode A model

Both models target the identical ONNX contract and the same controller, so they are
drop-in interchangeable. The difference is the proposer architecture:

- **flow** (`../diffusion_local`): conditional flow matching, integrates an ODE over
  a few Euler steps from sampled noise; the K candidates come from K fixed latents.
- **transformer** (this model): a strided conv tokenizes the costmap patch (with
  learned positional embeddings), a context token is prepended, and **K learned
  query tokens cross-attend** to that memory through two multi-head decoder blocks,
  each decoding a full trajectory in **one forward pass with no sampling noise**.
  The candidates differ because the queries differ. Its loss is a direct MSE onto
  the smooth pure-pursuit-arc expert, so the output is smooth and within the
  kinematic gate by construction (no `sample_weight` / integration-steps tuning).

## Intended use

Demonstration / research. Like the flow sibling it proves the *propose → safety-gate
→ score → cmd_vel* controller architecture with a real learned proposer that reads
the egocentric costmap and biases **every** candidate away from a one-sided
obstacle. It is the transformer member of the model zoo, added after the transformer
family ranked **2nd of eight configurations** in the offline leaderboard.

## Out-of-scope / limitations

Honest and load-bearing (same competence envelope as the flow sibling):

- **Synthetic data only.** Never validated on a real robot or rosbag.
- **One-sided obstacles only.** A symmetric head-on block is ambiguous (never seen);
  breaking that symmetry is a reactive-controller job (VFH+/ND).
- **Short, gentle.** ~1 s / ~0.3 m horizon with ~0.13 m lateral veer; it leans on
  closed-loop replanning rather than one big swerve.
- **Small research model.** Expected to reach the goal closed-loop in the *open*
  scenario; on obstacle scenarios (out of training distribution) the safety layer
  stops it safely short rather than threading — mature reactive controllers thread
  through. Threading needs better data / capacity (see
  [../../docs/generative_limits.md](../../docs/generative_limits.md) and
  [../../docs/next_phase.md](../../docs/next_phase.md)).
- **Carrot distribution.** Trained on carrots ~0.9–1.1 m ahead, bearing ±0.4 rad.
- **No safety authority.** The kinematic + footprint filters gate every proposal; if
  none is safe the controller falls back or stops. Do not deploy on hardware.

## Training data

`nav2_diffusion_training.generative_planners.make_costmap_dataset` (240 samples) —
identical to the flow Mode A model: one-sided obstacle band on +y (low cols, matching
`cropEgocentricPatch`) or −y of the 32×32 patch with varied row-band / column width,
emitted as mirrored +y/−y pairs (no lateral bias) plus clear samples; the carrot is
varied in distance (~0.9–1.1 m) and bearing (±0.4 rad); the expert
(`_expert_trajectory`) follows a pure-pursuit arc toward the carrot with a half-sine
bow away from the obstacle and path-tangent yaw. Fully procedural; no real data.

## Benchmark results

Research placeholder → checked **behaviourally**, not against the full Nav2
deployment benchmark:

- **Exported-model behaviour (onnxruntime):** obstacle on +y/left → all 3 candidates
  veer −y (mean lateral −0.13 m); obstacle on −y/right → all veer +y (+0.13 m); clear
  → centred (~0); per-step forward ~0.03 m (= 0.3 m/s × 0.1 s, within the kinematic
  gate); finite, smooth, monotone forward.
- **Offline leaderboard:** `costmap-transformer` ranks **2nd of 8** in
  [../../docs/model_comparison.md](../../docs/model_comparison.md) (success 1.00, zero
  collisions, single-step inference), just behind `costmap-consistency`.
- **End-to-end C++ side-selection:** asserted in `nav2_diffusion_onnx`'s
  `test_onnx_trajectory_model` (`CuratedZooTransformerVeersAwayFromObstacle`, runs
  where onnxruntime is available).
- **Collisions:** gated by the deterministic kinematic + footprint safety layer (0).

## Safety

- The model has **no** safety authority. Every proposal passes the kinematic and
  footprint filters; if none is safe the controller delegates to its
  `fallback_controller_plugin` or stops — never an unsafe command.
- Not for hardware use; no ODD validation performed.

## Reproducibility

- **Command:** `PYTHONPATH=../../nav2_diffusion_training python3 export.py`
  (uses CUDA when available; `CUDA_VISIBLE_DEVICES= ` forces a deterministic CPU build)
- **Seed:** 0 (`torch.manual_seed(0)` inside the training function)
- **Toolchain:** torch 2.10.0+cu128, onnx 1.21.0; trained on CUDA, exported on CPU
- **Hyperparameters:** transformer, 240 samples, 2000 epochs, lr 0.01 (the
  `recon_loss` is already a direct MSE onto the smooth expert, so no `sample_weight`
  or flow-step tuning is needed)
- Bit-for-bit reproduction may vary across torch versions / hardware (and CPU vs
  GPU); behaviour (side-selection, veer magnitude) is stable.

## License

Model: Apache-2.0 · Data: Apache-2.0 (procedurally generated here) · Code: Apache-2.0
