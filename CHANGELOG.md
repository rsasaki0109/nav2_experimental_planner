# Changelog

All notable changes to this project are documented here. The project aims to
follow [Semantic Versioning](https://semver.org/); APIs are not yet stable
before 1.0.0 (see [docs/roadmap.md](docs/roadmap.md)).

## [Unreleased]

### Added

- **Generative GlobalPlanner (Nav2 Mode B)** — `nav2_diffusion_global_planner`,
  a `nav2_core::GlobalPlanner` plugin. A model proposes K candidate start→goal
  paths via a new `nav2_diffusion_core::PathModel` seam (built-in analytic
  `FanPathModel`: straight line + symmetric detour fan; learned models loadable
  via pluginlib), a deterministic validity layer checks each candidate against
  the global costmap, and the shortest collision-free path is returned —
  otherwise `NoValidPathCouldBeFound` (or `StartOccupied`/`GoalOccupied`/
  `PlannerCancelled`). A targeted survey confirmed no open-source generative
  model is integrated as a Nav2 GlobalPlanner; the novelty is the Nav2-native
  integration + costmap validation/fallback wrapper. Closed-loop integration
  tests run against a live `Costmap2DROS` (no Gazebo/GPU): straight path on a
  clear map, detour around a partial obstacle, and failure on a full wall.
- **Learned generative path model for Mode B** — `PathFlowPlanner`
  (`nav2_diffusion_training.path_planners`), a flow-matching model that proposes
  K multimodal start→goal paths in a goal-aligned frame and exports to the ONNX
  contract `context [1,2] -> paths [1,K,H,2]`; plus `OnnxPathModel`
  (`nav2_diffusion_onnx`), a `nav2_diffusion_core::PathModel` ONNX backend that
  runs it, rotates/translates each candidate back into the map frame, and snaps
  endpoints onto start/goal. Set the planner's `model_plugin` to
  `nav2_diffusion_onnx::OnnxPathModel` to use it (planner stays free of any
  onnxruntime link, via pluginlib). Verified end to end in C++ gtests.
- **Costmap-conditioned generative path model for Mode B** —
  `CostmapPathFlowPlanner` (`path_planners`) reads a goal-aligned costmap patch
  and proposes K paths that bias toward the obstacle-free side, exported as a
  two-input ONNX (`context [1,2]` + `costmap [1,1,S,S]` -> `paths [1,K,H,2]`).
  `OnnxPathModel` auto-detects the `costmap` input and resamples the global
  costmap (passed by the planner) into the goal-aligned patch; the planner gains
  a `provide_costmap` param. C++ gtest confirms candidates veer away from the
  obstacle side with endpoints still anchored. The global analogue of the local
  costmap-conditioned controller — no OSS equivalent for Nav2.
- **Offline model-comparison leaderboard** — `tools/benchmark_models.py` +
  `nav2_diffusion_training.model_eval` (torch-free metrics: clearance, collision,
  progress, turning, success, safety-first ranking) trains the six generative
  families on a shared obstacle-with-gap dataset and writes
  `docs/model_comparison.md`. Costmap-conditioned models top the ranking;
  `costmap-consistency` (1-step) wins.
- **Costmap-conditioned avoidance demo GIF** (`docs/costmap_demo.gif`,
  `tools/costmap_demo.py`) rendered from the real `CostmapFlowPlanner`.
- **Foxglove visualization** — `nav2_diffusion_bringup/launch/foxglove.launch.py`
  (starts `foxglove_bridge` + the candidate/safety marker converter) and a
  ready-to-import Foxglove Studio layout
  (`foxglove/nav2_diffusion_layout.json`: 3D candidates/costmap/plan, SafetyState
  raw + state-transitions, cmd_vel plot). New `docs/visualization.md` covering
  both Foxglove and RViz.

### Changed

- `DiffusionPlanner` / `CostmapDiffusionPlanner` clamp the DDIM `x0` estimate
  (standard static thresholding) so sampling stays numerically stable instead of
  dividing by the near-zero final `alpha_bar`.

## [0.3.0] - 2026-06-03

Theme: **research-driven generative model families + costmap conditioning.** A
deep literature/OSS survey found no open-source flow-matching / diffusion /
consistency LOCAL planner for ROS 2 Nav2 ground robots; this release implements
all three behind the model seam, with optional costmap conditioning.

### Added

- **Three generative planner families** (`nav2_diffusion_training.generative_planners`):
  `FlowMatchingPlanner` (single/few-step conditional flow matching),
  `DiffusionPlanner` (cosine DDPM + DDIM), and `ConsistencyPlanner` (one-step
  distillation). Each maps a context vector to K SE(2) trajectories and exports a
  single self-contained ONNX file matching the `[1,4] -> [1,K,H,3]` backend
  contract. A trained flow model runs in the real C++ `OnnxTrajectoryModel`.
- **Costmap+goal conditioning end to end** (the surveyed OSS gap): `ModelContext`
  carries an optional egocentric costmap patch; `OnnxTrajectoryModel`
  auto-detects an optional `costmap` input and feeds `[1,1,S,S]`; the controller
  crops the patch via a new `costmap_patch_size` param (default off, fully
  backward compatible); `CostmapFlowPlanner` / `CostmapDiffusionPlanner` /
  `CostmapConsistencyPlanner` train and export two-input ONNX models.
- **README hero GIF** rendered from the real shipped pipeline (FanRolloutModel →
  footprint safety gate → scorer) showing obstacle avoidance with clearance, plus
  a data-generation-to-execution diagram.
- **GPU/headless Gazebo notes** in the bringup docs (NVIDIA EGL vendor +
  `GZ_SIM_RESOURCE_PATH`).

### Verification

- ROS 2 Jazzy: all packages build and test green (gtest + pytest + ament lints);
  the training round trip (expert/rosbag → dataset → PyTorch → ONNX → C++ backend)
  and the costmap two-input path are covered by tests. Live multi-process Gazebo
  remains unverified in the dev sandbox (DDS inter-process limits); logic is
  covered by in-process integration tests.

## [0.2.0] - 2026-06-03

Theme: **learned models, end to end.** A real ONNX inference backend behind the
plugin seam, a training pipeline that produces models for it, richer
visualization, and a fuller benchmark suite.

### Added

- **`nav2_diffusion_onnx`** (optional): `OnnxTrajectoryModel` implementing
  `TrajectoryModel` via ONNX Runtime (§5.2/§7.2), exported as a pluginlib plugin.
  Builds only when onnxruntime is found; otherwise builds empty so a plain
  `colcon build` never fails.
- **Controller model plugin loading**: `TrajectoryModel::configure()` plus
  `model_plugin` / `model_path` params let `DiffusionController` load a learned
  model (e.g. ONNX) at runtime via pluginlib, without the controller or core
  linking any inference library. Default stays the built-in `FanRolloutModel`.
- **Training pipeline** (`nav2_diffusion_training`): `build_samples` (base-frame
  future-trajectory labels), `track_from_bag` (rosbag ingestion),
  `unicycle_to_goal` (rule-based expert), and `train`/`train_and_export` (PyTorch
  → ONNX) matching the backend I/O contract. The expert→dataset→train→export→
  backend round trip is verified by tests.
- **RViz visualization** (`nav2_diffusion_rviz_plugins`): candidate markers
  (best/safe/rejected, best highlighted, rejection-reason text) and a SafetyState
  text marker; wired into the demo launches.
- **Benchmark suite** (`nav2_diffusion_benchmarks`): task/safety/smoothness
  metrics, safety-first composite score, leaderboard, per-controller aggregation,
  YAML scenario definitions, and a `benchmark_runner` node (§9.3–9.6).

### Changed

- Controller proposal stage now goes through the `TrajectoryModel` seam
  (`FanRolloutModel` built in) and extracts cmd_vel from the best trajectory.

### Verification

- ROS 2 Jazzy: `colcon build`/`colcon test` across all packages, 0 failures
  (370+ tests incl. gtest, pytest, and ament lints; the ONNX backend built
  against onnxruntime 1.24.2 and the training round trip both pass locally).

## [0.1.0] - 2026-06-03

First tagged milestone: **Nav2-native generative local controller** with a
deterministic safety layer and a benchmark suite. Matches the v0.1 theme
"costmap-conditioned generative local controller" in the roadmap.

### Added

- **Architecture docs** (`docs/`): architecture, safety, training, benchmarking,
  simulation, deployment, roadmap, risks, getting_started, contributing,
  model_zoo.
- **`nav2_diffusion_msgs`**: `TrajectoryCandidate`, `TrajectoryCandidates`, and
  `SafetyState` message contracts (architecture §4.3/§4.4, safety §8.3).
- **`nav2_diffusion_core`**: time-indexed SE(2) `Trajectory` types, a
  `TrajectoryScorer` (progress + smoothness), and the `TrajectoryModel` plugin
  seam (§5.2) with a built-in `FanRolloutModel` placeholder for learned models.
- **`nav2_diffusion_safety`**: deterministic, GPU-independent safety filters —
  `KinematicLimitsFilter` and costmap-based `FootprintCollisionFilter` (§8.2).
- **`nav2_diffusion_controller`**: a `nav2_core::Controller` plugin
  (`DiffusionController`) implementing the propose → input-validity → safety gate
  → score → extract pipeline (Mode A), with stale-data runtime gating (§7.4) and
  delegation to a configurable fallback controller (MPPI/RPP) when no safe
  candidate exists (§8.4). Loads in Nav2 alongside MPPI/RPP.
- **`nav2_diffusion_bringup`**: full Nav2 params (FollowPath → DiffusionController
  swap) plus loopback and Gazebo closed-loop demo launches.
- **`nav2_diffusion_benchmarks`**: task / safety / smoothness metrics, a
  safety-first composite score and Markdown leaderboard (§9.4–9.6), a Markdown
  report, and a `benchmark_runner` node with a unit-tested `RunRecorder`.
- **OSS scaffolding**: Apache-2.0 license, GitHub Actions CI (build + test across
  all packages on ROS 2 Jazzy), model manifest and model-card templates, issue /
  PR templates, and an RFC template.

### Verification

- ROS 2 Jazzy: `colcon build` of all six packages succeeds; `colcon test`
  reports 0 errors / 0 failures across gtest plus the ament lints.
- The controller's closed-loop behavior (drive-forward, stop-on-collision,
  stop-on-stale-pose, multimodal turn selection, fallback delegation) is covered
  by integration tests against a live `Costmap2DROS`, without Gazebo/GPU.

### Known limitations

- The generative model is the analytic `FanRolloutModel` placeholder; no learned
  model is included yet. ONNX/TensorRT backends will plug in behind
  `TrajectoryModel` in a later release.
- Live Gazebo closed-loop and the benchmark runner's cross-process service
  round-trip were not validated in the development sandbox (no GPU rendering for
  simulated LiDAR; DDS discovery flakiness). The underlying logic is unit-tested.
- This is not a safety-certified product; see [docs/safety.md](docs/safety.md).

[0.3.0]: https://github.com/rsasaki0109/nav2_diffusion_planner/releases/tag/v0.3.0
[0.2.0]: https://github.com/rsasaki0109/nav2_diffusion_planner/releases/tag/v0.2.0
[0.1.0]: https://github.com/rsasaki0109/nav2_diffusion_planner/releases/tag/v0.1.0
