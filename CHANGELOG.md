# Changelog

All notable changes to this project are documented here. The project aims to
follow [Semantic Versioning](https://semver.org/); APIs are not yet stable
before 1.0.0 (see [docs/roadmap.md](docs/roadmap.md)).

## [Unreleased]

### Changed
- **Closed the transformer's gap trade-off by raising model capacity — the shipped
  Mode B transformer now threads BOTH the off-centre gap AND the dead-ahead gaps.**
  Following the negative result below (a centred rebalance only *shifted* the
  trade-off at the old small capacity), bumped the Mode B path transformer to
  dim 64 / 8 heads / 3 decoder blocks (from 32 / 4 / 2) and tri-mixed dead-ahead
  (`centred`) samples into `'both'`, then retrained. Verified in the real C++
  `planner_benchmark` (8 courses): `Diffusion (Mode B, transformer)` now solves
  *off-centre gap* **and** *centred* / *narrow* / *double gate* (plus *clear* /
  *side obstacle*) — the over-aim trade-off is gone. The remaining bound is the
  *far off-centre gap* (off-axis slot ~3 m forward), now *no path*; *slalom* stays
  hybrid-only. Re-exported the model (new checksum), updated `manifest.yaml`,
  `model_card.md`, `model_zoo/README.md`, `docs/generative_limits.md`,
  `README.md`, the `planner_benchmark` narrative, and regenerated
  `docs/planner_comparison.md`. onnx C++ gtests 10/10 (incl. the aim test).

### Added
- **`make_costmap_path_centred_gap_dataset` + a `'centred'` path dataset option**
  (dead-ahead slot, straight-through expert; includes narrow on-line slots), with a
  unit test. This is the data for the centred-rebalance experiment below; it is
  intentionally *not* mixed into `'both'`.

### Changed
- **Tried the documented centred-rebalance to close the transformer's gap
  trade-off — it only *shifts* it (an honest negative result, verified in C++).**
  Mixing centred-gap samples into `'both'` and retraining made the transformer
  gain *centred*/*narrow* but **lose the off-centre gap** (the project headline)
  and *double gate*; an off-centre-weighted mix lost off-centre too, and off-centre
  threading is flaky across GPU runs. So it is a genuine small-model capacity limit,
  not a data-mix oversight. `'both'` stays two-way, the **shipped model is unchanged**
  (off-centre headline preserved), and the finding is recorded in
  `docs/generative_limits.md` and the transformer `model_card.md`. Closing the
  trade-off needs more model capacity or curriculum / multi-task training.
- **Root-caused the sandbox "no live ROS" limit and corrected the record.** With the
  agent command sandbox disabled (foreground), DDS works on this very machine —
  multicast loopback (JOIN_OK, 2/2), demo talker→listener discovery (heard 9), and
  `tb3_gazebo_mission.launch.py` runs the full stack (Gazebo spawn → AMCL active →
  bt_navigator loaded). So the blocker was the **agent's command sandbox**, not the
  machine, ROS, or the project. The only remaining barrier is the nav2 composition's
  pathologically slow cold start in that context (~230 s to localization;
  repeated `load_node` service timeouts), which can outlast the mission's wait.
  Documented in `docs/simulation.md` §10.5 (追記3).
- **Raised the Gazebo mission node's server wait** — `tb3_gazebo_mission.launch.py`
  gains a `server_wait_sec` arg (default 180 s) since a cold Gazebo + Nav2
  composition start routinely exceeds the previous 60 s default.
- **Documented opening the demo MCAP in the Foxglove web app** (no install:
  app.foxglove.dev → Open local file) alongside the desktop app, in
  `docs/visualization.md`.
- **Refined the sandbox DDS diagnosis in `docs/simulation.md` §10.5.** Raw loopback
  UDP between two processes *does* work here (3/5 datagrams), so the blocker is not
  a hard IPC wall — it is the DDS discovery layer specifically: Fast DDS publishes
  but peers never discover each other (SHM / UDPv4 / localhost-only / explicit-port
  unicast all yield 0 received), and CycloneDDS cannot allocate a participant index.
  Conclusion unchanged (no live inter-process ROS 2 in this sandbox) but the cause
  is now precisely located; a real ROS host (or a discovery server) works.

### Added
- **Foxglove-openable MCAP recording of the real Mode B pipeline**
  (`tools/foxglove_mcap_demo.py` → `docs/mode_b_demo.mcap`). It runs the shipped
  `PathFlowPlanner`, sweeps an obstacle, and writes each frame's costmap +
  proposed/validated/selected paths as standard ROS 2 messages (`OccupancyGrid`,
  `Path`, `PoseArray`, `PoseStamped`; 24 frames / ~2.3 s) so Foxglove Studio can
  open it (3D panel, scrub/play, Export → Video) with no live ROS. The MCAP is
  validated structurally (round-tripped through the mcap_ros2 reader).
  `docs/visualization.md` documents opening it and recording a video. Honest
  scope: it records the *model pipeline* to a portable file — not a live
  Gazebo/ROS run (the sandbox has no display and blocks inter-process DDS, so a
  Foxglove screen-capture cannot be produced here).
- **Animated visualization of the closed-loop Gazebo courses** in the README
  (`docs/sim_courses.gif`, reproducible via `tools/gazebo_courses_demo.py`). It
  renders the `nav2_diffusion_sim` course layouts (centred / off-centre gap /
  slalom) from the generated occupancy grids with a valid start→goal route (grid
  A\*) and a new README section. Honest scope: it depicts the *courses*, not a
  closed-loop Gazebo run — real closed-loop numbers still need a real ROS host.

## [0.10.0] - 2026-06-05

Theme: **the off-centre-gap ceiling falls, and the evaluation gets honest and
closed-loop.** A differentiable footprint-clearance loss lets the Mode B
transformer thread the footprint-validated off-centre gap as a pure-generative
planner (the documented ceiling, broken) — while an expanded 8-course benchmark
surfaces the honest trade-off (it over-aims and misses dead-ahead gaps that flow /
recurrent thread; threading is *not* forward-distance bounded — that claim is
retracted). The closed-loop side grows up too: the Gazebo mission harness goes
from one goal to a multi-leg leaderboard with named presets, and
`nav2_diffusion_sim` ships single-source obstacle courses (world + map + goals
generated together) mirroring the off-line scenarios. The 21 packages are grouped
into role subdirectories (`generative/` / `classical_planners/` /
`reactive_controllers/` / `benchmarks/`; names unchanged, build unaffected), the
README is now English, and CI — red for 15+ commits from an invalid rosdep key —
is green again, now running inside the official `ros:jazzy` container.

### Fixed
- **Greened CI, which had been red for 15+ commits.** The root cause was an
  invalid `<buildtool_depend>ament_python</buildtool_depend>` in
  `nav2_diffusion_training` (`ament_python` is a build type, not a rosdep key):
  it made the CI rosdep step fail, so the rosidl typesupport packages were never
  installed and `nav2_diffusion_msgs` failed to configure — masking every later
  test. With the build unblocked, fixed the lint debt CI could finally reach:
  added `#include <vector>` to `diffusion_global_planner.hpp` (cpplint), uncrustify-
  reformatted `diffusion_global_planner.cpp`, and converted 20 multi-line docstrings
  in `nav2_diffusion_training` to D213 (second-line summary) style so `ament_pep257`
  passes while `ament_flake8` still does. (Locally-only false positives — a stray
  in-package `install/` dir tripping `ament_copyright`, and a colcon result-capture
  race on the JPS gtest — are not present in a fresh CI checkout.)

### Changed
- **Grouped the 21 ROS packages into role subdirectories** — `generative/`
  (the generative framework, `nav2_diffusion_*`), `classical_planners/`
  (RRT\* / RRT-Connect / PRM / D\* Lite / JPS / Lazy Theta\* / ARA\* / visibility
  graph), `reactive_controllers/` (VFH+ / ND), and `benchmarks/`
  (`nav2_planner_benchmarks` + `nav2_diffusion_benchmarks`). **Package names,
  dependencies, and imports are unchanged** — colcon discovers packages
  recursively, so the grouping is purely for readability and does not affect the
  build. Updated the references that *are* path-relative: the `../model_zoo` →
  `../../model_zoo` model paths in `nav2_planner_benchmarks` and
  `nav2_diffusion_onnx` CMake (the only build-breaking refs), README / docs
  links, the `model_zoo` reproduce commands, the architecture §12 package table,
  and added `nav2_diffusion_sim` to CI. Verified by a clean full rebuild (21/21)
  plus the onnx gtests (load models from the new path), `nav2_diffusion_sim`
  (30/30), `nav2_diffusion_bringup` (34/34), and an end-to-end `planner_benchmark`
  run whose results are identical to before the move.

### Added
- **Closed-loop Gazebo obstacle courses (`nav2_diffusion_sim`) mirroring the
  off-line `planner_benchmark` scenarios.** A single-source generator
  (`nav2_diffusion_sim/gen_courses.py`) defines each course once (map extent +
  start + goals + wall boxes) and emits three mutually-consistent artifacts: a
  self-contained gz-sim world (`worlds/<course>.sdf.xacro`, mirroring the stock
  `tb3_sandbox` plugin set so the LiDAR renders), a matching occupancy map
  (`maps/<course>.pgm` + `.yaml`, so AMCL / the global costmap see the same walls),
  and the mission goals. Three courses ship — `centred` (gap dead ahead), `gap`
  (~2 m off-centre), `slalom` (two staggered gaps) — driven by
  `tb3_gazebo_course.launch.py course:=<name>` (loads the world+map, spawns TB3 at
  the start, brings up Nav2 + DiffusionController, runs the mission, writes a
  leaderboard). This brings the multi-course evaluation to the **full closed-loop
  stack** (global planner + controller + costmap), complementing the proposal-stage
  `planner_benchmark`. The package is made a real `ament_cmake` package (was a
  skeleton README); geometry / artifact-consistency is **unit-tested**
  (`test/test_gen_courses.py`, 7 cases: start/goals clear, obstacle courses block
  the straight line, map↔walls↔goals consistent, committed artifacts match the
  spec). The closed-loop run needs a real ROS host (the sandbox blocks
  inter-process DDS), so no fabricated sim numbers are committed.
- **Strengthened the headless Gazebo closed-loop harness from a single goal to a
  multi-leg mission course with a leaderboard.** `sim_mission.py` now drives a
  *sequence* of named `NavigateToPose` legs (each sent from where the previous ended)
  and writes a Markdown leaderboard (one row per leg + a reached-count / total-path /
  total-time summary) — the closed-loop counterpart of the offline `planner_benchmark`
  sweep. Legs are passed via a `missions` string array (`"label|x|y|yaw|timeout"`);
  or via a named **course preset** (`course:=default|there_and_back|patrol`); with
  neither given it falls back to the single `goal_x`/`goal_y`/`goal_yaw` goal
  (backward compatible — precedence is `missions` > `course` > single goal).
  `tb3_gazebo_mission.launch.py` gained `;`-separated `missions`, a `course` preset
  argument, and a `stop_on_failure` flag. The leg-spec/course parsing, metric
  aggregation, and leaderboard formatting are refactored into pure (no-ROS) functions
  and **unit-tested** (`nav2_diffusion_bringup/test/test_sim_mission.py`, 12 cases wired
  into `colcon test` via `ament_add_pytest_test`); only the driving loop needs a live
  Nav2 + Gazebo. ROS imports are now lazy so the module imports without a ROS runtime.
  Course presets are sustained-nav goal sequences for the open `tb3_sandbox` world;
  obstacle courses mirroring the planner_benchmark gaps/slalom need SDF walls (future
  work). Updated `docs/simulation.md` section 10.5. (The sandbox still cannot run the
  closed-loop sim — inter-process DDS is blocked — so no fabricated sim numbers are
  added; the harness produces the leaderboard on a real ROS host.)
- **Expanded the Mode B planner benchmark from 4 to 8 courses** (`centred gap`,
  `narrow gap`, `far off-centre gap`, `double gate` join `clear` / `off-centre gap` /
  `slalom` / `side obstacle`) in `nav2_planner_benchmarks/planner_benchmark.cpp`, and
  regenerated `docs/planner_comparison.md`. The sweep surfaced two honest findings that
  correct the shipped scope: (1) the off-centre-trained transformer is a
  **specialization, not a strict upgrade** — it *over-aims* and reports **no path on the
  centred / narrow dead-ahead gaps** that the flow and recurrent siblings thread
  trivially (the three families are complementary, not ordered); (2) gap threading is
  **not bounded to the ~2 m training forward-distance** — the transformer also threads
  the `far off-centre gap` (wall ~3 m forward), so the earlier "bounded forward-distance"
  claim is **retracted**. Updated `docs/generative_limits.md`, the transformer
  `model_card.md` / `manifest.yaml`, `model_zoo/README.md`, and the benchmark narrative
  accordingly.
- **Differentiable footprint-clearance loss for Mode B path training**
  (`nav2_diffusion_training.path_planners._footprint_penalty`, exposed via
  `train_and_export_costmap_path(..., footprint=, blur_sigma=, inflate_cells=)`).
  It samples a Gaussian-blurred obstacle-proximity field along the densely
  interpolated candidate path and penalizes overlap, optimizing the proposals to be
  *what the deterministic validity layer accepts* rather than only imitating an
  expert. The blur supplies a gradient even in the wall interior (where a raw
  occupancy penalty is flat); the dense interpolation samples the wall crossing the
  way the C++ `isPathValid` does. Python tests:
  `test_footprint_penalty_prefers_routing_through_the_slot`,
  `test_footprint_training_lowers_clearance_vs_recon_only`,
  `test_footprint_loss_rejected_for_flow_kind`.

### Changed
- **Mode B transformer now threads the footprint-validated off-centre gap as a
  pure-generative planner — the documented gap ceiling is broken.** Retraining
  `diffusion_global_costmap_transformer_v0` with the footprint-aware loss (token
  attention *aims* at the off-centre slot; the footprint term pulls each candidate's
  wall crossing into the free slot with margin) makes the real C++ `planner_benchmark`
  report `Diffusion (Mode B, transformer)` *off-centre gap* = **yes (~5.5 m, 12-pose
  generative path, ~0.2 ms, no fallback)** — the first Mode B model here to thread the
  1 m footprint-validated slot without a classical fallback. It keeps the *clear* /
  *side obstacle* competence; the flow and recurrent Mode B models still report
  *no path* on the gap. Updated the model artifact + checksum, `model_card.md`,
  `manifest.yaml`, `export.py` (footprint 3.0 / blur_sigma 2.5, 240 samples / 2500
  epochs), the `planner_benchmark` narrative + family label, and regenerated
  `docs/planner_comparison.md`. Rewrote the off-centre-gap section of
  `docs/generative_limits.md` to record the breakthrough and its honest scope:
  *slalom* (two-crossing S) is still no-path for pure generative, and the hybrid planner
  remains the any-map completeness guarantee. (The 8-course sweep above later refined
  this scope — see the Added entry.)

## [0.9.0] - 2026-06-05

Theme: **generative family expansion across both seams.** v0.6.0–v0.8.0 put the first
learned models in the loop and made generative+classical complete via hybrids; v0.9.0
broadens *what* proposes. Two new generative families — **transformer** (DETR-style
set-prediction, one deterministic forward pass) and **recurrent** (GRU autoregressive
rollout, one waypoint at a time) — are implemented and shipped on **both** the Mode A
local-trajectory (`OnnxTrajectoryModel`) and Mode B global-path (`OnnxPathModel`)
contracts, taking Mode A to five families (flow / diffusion / consistency /
transformer / recurrent) and Mode B to three (flow / transformer / recurrent). Each
new family ships a curated, GPU-trained, CPU-exported `model_zoo` artifact running
through the real C++ inference path, guarded by a curated-zoo C++ test and added to
the benchmarks (`controller_benchmark` / `planner_benchmark`) and the offline
leaderboard (now ten configurations). Honest scope: these are same-contract *peers*
that compare inductive biases on identical scenarios — the transformer's distinct
property is a *representational* one (its proposals **aim** at an off-centre slot
where the flow model's CNN embedding cannot), which is verified by an A/B probe and a
C++ direction test but still does **not** thread the narrow footprint-validated gap;
the synthetic-data / capacity ceiling and the hybrid completeness guarantee are
unchanged. Also adds an in-launch, discovery-free **Gazebo mission harness**
(`sim_mission.py`) so closed-loop sim numbers can be produced on a real ROS host.

### Added
- **Recurrent (GRU) rollout Mode B global-path family** (`CostmapPathRecurrentPlanner`
  in `nav2_diffusion_training.path_planners`) — the **third** generative family on the
  `OnnxPathModel` (Mode B) contract, alongside flow and transformer, bringing the
  recurrent family to the global-path seam too. A 16-d CNN embedding of the goal-aligned
  patch plus the goal-distance context conditions a GRU that emits each path one
  waypoint at a time; K=5 learned seed vectors make the candidates distinct and they are
  trained as a lateral fan around the routing expert so the footprint validator gets a
  spread. Shipped in the loop as `model_zoo/diffusion_global_recurrent/`
  (`diffusion_global_costmap_recurrent_v0`, ≈1.0 MB, GPU-trained, CPU-exported, final
  loss 0.0025); a curated-zoo C++ test (`CuratedZooPathRecurrentVeersAwayFromObstacle`)
  guards the shipped binary and a `Diffusion (Mode B, recurrent)` row is added to
  `planner_benchmark` (regenerated into `docs/planner_comparison.md`). Honest scope:
  conditioning on a CNN embedding (like the flow model, **not** the transformer's token
  attention) makes it a **benchmark peer of the flow Mode B model** — it picks the free
  side of a one-sided obstacle (clears *clear* / *side obstacle*, *no path* on
  *off-centre gap* / *slalom*) and does **not** aim at an off-centre slot; the
  gap-routing ceiling and the hybrid completeness guarantee are unchanged. Cost: the
  H=12 autoregressive rollout is the highest-latency Mode B family (flow 4 steps,
  transformer 1 forward). Demonstrates the seam carries the same sequential family on
  both the Mode A and Mode B contracts.
- **Recurrent (GRU) rollout trajectory model family** (`RecurrentRolloutPlanner` /
  `CostmapRecurrentPlanner` in `nav2_diffusion_training.generative_planners`) — the
  **fifth** generative family on the `OnnxTrajectoryModel` contract, alongside flow /
  diffusion / consistency / transformer. Unlike the one-shot / denoising families it
  generates the trajectory **autoregressively**: a CNN encodes the costmap patch to a
  conditioning vector and a GRU emits the SE(2) points one at a time, feeding the
  previous point back in (the world-model-style sequential inductive bias). K=3
  learned seed vectors make the candidates distinct without sampling noise; the
  `HORIZON`/`K` loops unroll into a static graph so the GRU exports cleanly to ONNX
  (no custom ops). Loads into the existing C++ backend with no changes (same
  `context [1,4]` + `costmap [1,1,S,S]` → `[1,K,H,3]` contract). No surveyed work
  open-sources a recurrent-rollout local trajectory planner integrated with Nav2.
  Tests cover the ONNX contract and verify the costmap-conditioned model veers away
  from a one-sided obstacle and rolls forward over the horizon.
- **Recurrent Mode A model shipped in the loop** — `model_zoo/diffusion_local_recurrent/`
  (`diffusion_local_costmap_recurrent_v0`, ≈592 KB, GPU-trained, CPU-exported). The
  recurrent family is now a real learned model running through the C++ inference path:
  a curated-zoo C++ test (`CuratedZooRecurrentVeersAwayFromObstacle`) guards the
  shipped binary, a `Diffusion (Mode A, recurrent)` row is added to
  `controller_benchmark`, and the family is added to the offline leaderboard
  (`tools/benchmark_models.py`, now ten configurations) and regenerated into
  `docs/model_comparison.md`, where `costmap-recurrent` tops the safety-weighted
  score (success 1.00, zero collisions) — though at 10 sequential rollout steps it
  is the highest-latency family, so the 1-step consistency / transformer remain the
  edge-GPU picks. Exported behaviour: obstacle on +y → all candidates
  veer −y (≈−0.13 m), symmetric on −y, clear → straight, monotone forward rollout to
  ~0.30 m. Honest scope: a fifth same-contract generative proposer (sequential bias),
  not a new gap/threading capability — the synthetic-data / capacity ceiling and the
  hybrid completeness guarantee are unchanged.
- **Transformer Mode B path planner + off-centre-gap finding (representational, not a
  benchmark win).** `CostmapPathTransformerPlanner` + `make_costmap_path_gap_dataset`
  in `path_planners.py`, and `model_zoo/diffusion_global_transformer/`
  (`diffusion_global_costmap_transformer_v0`, GPU-trained). Direct A/B on the same
  off-centre-gap data: the flow Mode B model cannot aim a proposal at the slot
  (loss 0.12, near-straight / wrong side); the transformer's attention over costmap
  tokens **aims every proposal at the slot on both sides** (loss 0.002, lateral at the
  wall ≈ ±2 m), guarded by `OnnxPathModelTest.CuratedZooTransformerAimsAtOffCentreSlot`.
  **Honest scope:** a *proposal-direction* advance, not a gap-solving win. The K
  candidates are trained as a small lateral fan (the flow model gets this spread from
  its K fixed latents), which makes it a **peer of the flow model on the
  `planner_benchmark`** — clears *clear* + *side obstacle*, *no path* on *off-centre
  gap* / *slalom* — and it is shipped as a `Diffusion (Mode B, transformer)`
  leaderboard row. Its distinct property is that its proposals *aim* at the off-centre
  slot where flow's cannot, but that aim still does not thread the narrow 1 m
  footprint-validated slot; the hybrid planner remains the completeness guarantee for
  the gap. See `docs/generative_limits.md`.
- **Transformer trajectory model family** (`TransformerPlanner` /
  `CostmapTransformerPlanner` in `nav2_diffusion_training.generative_planners`) —
  the fourth generative family on the `OnnxTrajectoryModel` contract, alongside
  flow / diffusion / consistency. A DETR-style set-prediction decoder: K learned
  query tokens cross-attend to a context token (plus tokenized costmap patch for
  the costmap-conditioned variant) and each decodes a full SE(2) trajectory in a
  single deterministic forward pass — multimodality comes from the distinct
  queries, not from sampling noise. Self-contained attention (no
  `nn.MultiheadAttention`) so the ONNX export is small and backend-agnostic; loads
  into the existing C++ backend with no changes (same `context [1,4]` +
  `costmap [1,1,S,S]` → `[1,K,H,3]` contract). No surveyed work open-sources a
  transformer local trajectory planner integrated with Nav2. Tests cover the ONNX
  contract for both variants and verify the costmap-conditioned model learns to
  veer away from a one-sided obstacle.
- Transformer added to the offline leaderboard (`tools/benchmark_models.py`, now
  eight configurations). On the shared synthetic avoidance scenarios
  `costmap-transformer` ranks **2nd** (success 1.00, zero collisions, single-step
  inference), just behind `costmap-consistency`, and `transformer` is the best of
  the context-only models — regenerated into `docs/model_comparison.md`.
- **Transformer Mode A model shipped in the loop** — `model_zoo/diffusion_local_transformer/`
  (`diffusion_local_costmap_transformer_v0`, ≈224 KB, GPU-trained, CPU-exported).
  The transformer family is now a real learned model running through the C++
  inference path, not just training code: a curated-zoo C++ test
  (`CuratedZooTransformerVeersAwayFromObstacle`) guards the shipped binary, and a
  `Diffusion (Mode A, transformer)` row in `docs/controller_comparison.md` shows it
  reaching the goal closed-loop in the open scenario (equivalent to the flow Mode A
  model — confirming the obstacle-scenario ceiling is the synthetic data / capacity,
  not the model family).
- GPU training support: `train_and_export_costmap(..., device=...)` trains on the
  given device (e.g. `'cuda'`) and always exports on CPU for a portable artifact.

- **Gazebo closed-loop mission harness** — an in-launch, external-discovery-free
  way to run the headless TB3 sim end-to-end:
  - `nav2_diffusion_bringup/scripts/sim_mission.py` — a mission node that waits for
    Nav2's `navigate_to_pose`, drives one goal, records executed odometry, and
    writes a Markdown result file (the file, not a topic, is the artifact).
  - `launch/tb3_gazebo_mission.launch.py` — brings up the sim + mission node and
    shuts the launch down when the mission finishes.
  - `params/nav2_diffusion_tb3.yaml` — AMCL `set_initial_pose` so localization
    activates headless (no RViz 2D Pose Estimate needed).
  - `config/fastdds_udp_localhost.xml` — a Fast DDS UDP-only / unicast-localhost
    profile for sandboxes without SHM or multicast.
  Built and lint-clean; the mission ran and wrote its result file. The closed-loop
  *numbers* are still blocked in this sandbox by a complete inter-process DDS
  failure (even a 2-process talker/listener hears nothing under every transport
  tried) — documented exhaustively in `docs/simulation.md` §10.5. On a DDS-capable
  host the mission launch produces the sim numbers directly.

### Documentation
- `docs/simulation.md` §10.5 — verified headless Gazebo bring-up. An actual run of
  `tb3_gazebo_diffusion.launch.py` confirms gz renders the GPU LiDAR (`/scan`
  count 360, the `libEGL ... dri2` warning is non-fatal), the ros_gz bridges and
  the Nav2 stack load, and `FollowPath` is the DiffusionController. Documents the
  two blockers to a fully automated closed-loop numbers run in a restricted
  sandbox (localization needs an initial pose; the DDS layer rejects external
  processes — Fast DDS `/dev/shm` lock failure, CycloneDDS participant-index
  failure) and the path to finish it (an in-launch mission node + a DDS-capable
  host). `docs/next_phase.md` 段3 cross-references it.
- `docs/next_phase.md` — an execution plan for the data-/environment-dependent
  next phase. Consolidates the future-work threads scattered across the roadmap and
  `generative_limits.md` into prerequisites → ordered stages (faithful closed-loop
  DAgger, larger model + real data, real-robot shadow mode, TensorRT/Jetson) → each
  with a definition of done, mapped onto the existing scaffolding. Linked from the
  README doc index and the roadmap.

## [0.8.0] - 2026-06-04

Theme: **deeper coupling and closed-loop training.** v0.7.0 made generative+classical
complete via a *fallback* hybrid (classical only when the learned proposals fail).
v0.8.0 explores the rest of that design space — tighter coupling and learning on the
states the policy actually visits. The Mode B planner gains a **tightly-coupled
guided** hybrid (`hybrid_mode: guided`): a built-in complete A* whose cell costs are
discounted near the valid proposals, so the learned model *shapes every route* while
the search guarantees completeness. And the training side gains a **DAgger** closed-
loop loop (`dagger.py`) that rolls the policy out in a costmap sim, relabels visited
states with an expert, and retrains — the principled fix for the distribution shift
documented in `docs/generative_limits.md`. Honest scope: the guided hybrid solves
every benchmark scenario; DAgger is committed as reusable infrastructure whose gain
is marginal with the small model, pointing the way to bigger models + a faithful loop.

### Added

- **DAgger closed-loop training loop for the Mode A trajectory model**
  (`nav2_diffusion_training/dagger.py`). Targets the distribution shift documented
  in docs/generative_limits.md: it rolls the current policy out in a lightweight
  numpy costmap sim (mirroring the C++ controller's egocentric crop, first-segment
  command extraction, nearest-then-forward lookahead, and dt), queries an expert
  (pure-pursuit toward the carrot + costmap-read avoidance) at every visited state,
  aggregates those (state, expert) labels, and retrains — exporting the standard
  `[1,4]+[1,1,32,32] -> [1,3,10,3]` ONNX. The loop runs end to end (pytest
  `test_dagger.py`), but with the small synthetic model + lightweight sim the
  closed-loop gain is marginal (0/4 → 1/4 reached; the aggregated set outgrows the
  tiny model's capacity). It is committed as **reusable infrastructure** for the
  documented future work (bigger models + a faithful real C++/Gazebo closed loop),
  not as a shipped model — the curated Mode A model is unchanged and Mode A
  obstacles are already solved by the hybrid. See docs/generative_limits.md.
- **Tightly-coupled hybrid Mode B planner (`hybrid_mode: guided`).** Beyond the
  fallback hybrid (which only invokes a classical planner when the generative
  proposals all fail), `DiffusionGlobalPlanner` gains a built-in complete 8-connected
  A* that discounts the cost of cells near the valid proposals (`guidance_strength` /
  `guidance_radius`). The learned model *shapes every route* (which way to go around)
  while the search guarantees completeness; with no valid proposal it degrades to a
  plain complete A*. `planner_benchmark` adds a *Diffusion (Mode B, guided)* entry
  that solves every scenario at cell resolution (81 / 111 poses) — unlike the
  fallback hybrid which returns the 12-pose generative path on easy maps. New gtest
  `GuidedHybridSolvesSlalomWithCompleteSearch` (no external fallback plugin needed).
  See docs/generative_limits.md.

## [0.7.0] - 2026-06-04

Theme: **hybrid — generative proposes, classical disposes (and completes).** v0.6.0
put the first learned models in the loop and was honest that small synthetic models
hit a ceiling on gap-routing (Mode B) and obstacle-threading (Mode A). v0.7.0 breaks
that ceiling *architecturally* rather than by overfitting a tiny model: each mode
gains a classical fallback, so the learned model proposes (fast, multimodal,
costmap-biased on the easy cases) and a complete classical method disposes by
guaranteeing a result on the hard ones. The Mode B planner falls back to a classical
search (JPS); the Mode A controller falls back to a classical reactive controller
(VFH+). Both now **reach the goal / return a valid path in every benchmark
scenario** via propose→dispose, and `docs/generative_limits.md` documents where the
learned models help, where they hit their ceiling, and how the hybrid passes it.

### Added

- **Hybrid Mode A controller (generative propose → classical reactive dispose).**
  `controller_benchmark` adds a *Diffusion (Mode A, hybrid)* entry: the learned
  `DiffusionController` with `fallback_controller_plugin` set to VFH+ (the param
  already existed; this exercises it). When no learned candidate is safe it
  delegates to the classical reactive controller instead of stopping, so it
  **reaches the goal in every scenario** — learned drives *open*, VFH+ threads the
  obstacle scenarios (its corridor row matches VFH+ exactly, confirming the
  fallback drove). This is the symmetric Mode A analogue of the hybrid Mode B
  planner; both modes now complete every benchmark scenario via propose→dispose.
- **Hybrid Mode B planner (generative propose → classical search dispose).**
  `DiffusionGlobalPlanner` gains a `fallback_planner_plugin` parameter: when no
  generative candidate is collision-free it delegates to a classical, complete
  `nav2_core::GlobalPlanner` (mirroring the controller's `fallback_controller_plugin`)
  instead of throwing. This breaks the learned model's gap-routing ceiling
  *architecturally* — the planner uses the fast learned proposal on easy maps but
  never regresses below a search planner on hard ones. `planner_benchmark` adds a
  *Diffusion (Mode B, hybrid)* entry (learned + JPS fallback) that **solves every
  scenario**: clear / side-obstacle via the generative path (12 poses), off-centre
  gap / slalom via the JPS fallback (81 / 107 poses). New integration test
  `HybridFallsBackToClassicalSearch` (analytic/learned alone throws on a slalom; the
  hybrid returns a complete collision-free path). See docs/generative_limits.md.
- **`docs/generative_limits.md`** — an empirical note mapping where the small
  synthetic learned models help (costmap side-selection in both modes; Mode A
  open-scenario goal reaching) versus where they hit their ceiling (Mode B
  off-centre-gap routing; Mode A obstacle threading), why (tiny-CNN brittleness,
  synthetic→real patch transfer, single-shot vs search, closed-loop distribution
  shift), and the concrete paths past it (bigger models + diverse real data,
  closed-loop/DAgger training, generative-propose → classical-refine hybrid).
  Documents the exploration behind v0.6.0's honest benchmark results. Linked from
  the README docs map and the selection guide.

## [0.6.0] - 2026-06-04

Theme: **the first learned models in the loop.** v0.4.0 added classical breadth and
v0.5.0 made the catalog usable; this release makes the *generative* charter real.
Until now the "Learned models propose. Classical safety disposes. Nav2 executes."
tagline was carried entirely by analytic placeholders — no trained model ran
anywhere. v0.6.0 ships the first two curated learned models, both end-to-end through
the real C++ ONNX inference path (not unit-test fixtures): a costmap-conditioned
flow **path** model (Mode B global planner) and a costmap-conditioned flow
**trajectory** model (Mode A local controller). Both read the costmap and bias every
proposal to the open side; the deterministic layers gate them. The model zoo is now
populated (with manifests, model cards, and reproducible export scripts), both
comparison benchmarks run a learned entry, and the learned Mode A controller reaches
the goal closed-loop in the open scenario. Honest throughout about where the small
synthetic models hit their ceiling — the architecture works end-to-end; the models
are the limit.

### Added

- **Learned model in the loop for Mode A (local controller).** The headline
  "learned models propose trajectories" mode now has a curated trained model:
  [`model_zoo/diffusion_local/`](model_zoo/diffusion_local) ships a
  costmap-conditioned flow **trajectory** model (`costmap_flow.onnx`, ≈268 KB, with
  manifest / model card / reproducible `export.py`) that runs end-to-end through
  `nav2_diffusion_controller::DiffusionController` →
  `nav2_diffusion_onnx::OnnxTrajectoryModel`. It reads the egocentric costmap and
  biases every candidate away from a one-sided obstacle (asserted end-to-end in
  `nav2_diffusion_onnx`'s `test_onnx_trajectory_model`). The closed-loop
  `controller_benchmark` now runs it as *Diffusion (Mode A, learned)* beside VFH+
  and ND: it **reaches the goal closed-loop in the open scenario**, and on the
  obstacle scenarios (out of its training distribution) the safety layer stops it
  safely short of the block rather than threading it (no collision) — where the
  mature reactive controllers thread through. Honest demonstration that the
  *architecture* works end-to-end; threading obstacles needs a better model.
- **Carrot-directed costmap trajectory dataset.**
  `nav2_diffusion_training.generative_planners.make_costmap_dataset` now varies the
  carrot distance/bearing and the expert follows a **pure-pursuit arc toward the
  carrot** (the key to closed-loop goal tracking) plus an avoidance bow, and
  `train_and_export_costmap` gained `steps` / `sample_weight` options (a direct MSE
  to the smooth expert) so the sampled trajectory stays smooth and within kinematic
  limits. Backward-compatible defaults.
- **First learned model in the loop (Mode B).** The repo's generative claim is no
  longer carried only by analytic placeholders: a curated costmap-conditioned
  flow-matching path model now ships in [`model_zoo/diffusion_global/`](model_zoo/diffusion_global)
  (`costmap_flow.onnx`, ≈349 KB, with `manifest.yaml`, `model_card.md`, and an
  `export.py` that reproduces it) and runs end-to-end through the real C++ ONNX
  inference path (`nav2_diffusion_onnx::OnnxPathModel`), not just a unit-test
  fixture. It reads the costmap and biases **every** proposal toward the
  obstacle-free side (no built-in left/right bias; clear → centred). This is the
  repo's first populated model-zoo entry.
- **Learned Mode B in the planner comparison.** `planner_benchmark` now also runs
  the learned model (*Diffusion (Mode B, learned)*) alongside the analytic fan and
  the eight classical planners, and a new **side-obstacle** scenario plays to its
  costmap-conditioned competence. It clears *clear* and *side obstacle* but reports
  *no path* on *off-centre gap* (a 2 m detour) and *slalom* (an S-shape): the
  ceiling is the synthetic training distribution, not the architecture — richer
  data lifts it and the same deterministic validity layer still gates the output.
  See [docs/planner_comparison.md](docs/planner_comparison.md).
- **End-to-end C++ test for the curated artifact.**
  `nav2_diffusion_onnx`'s `test_onnx_path_model` loads the shipped `model_zoo`
  binary and asserts the costmap side-selection behaviour (obstacle-left → all
  candidates veer right, and vice versa), guarding the artifact against drift
  (runs where onnxruntime is available).

### Changed

- **`nav2_diffusion_training.path_planners.make_costmap_path_dataset`** now varies
  obstacle width / forward extent, emits mirrored +y/−y pairs, and includes clear
  (no-obstacle) samples, so the learned Mode B model responds symmetrically and
  without a built-in lateral bias.

### Fixed

- **Lookahead carrot could point backwards in `DiffusionController`** (the same bug
  fixed earlier in VFH+/ND): it scanned the plan from the start for the first pose
  beyond `lookahead_distance`, so once the robot advanced the carrot fell back onto
  passed poses behind it. It now finds the nearest plan pose and looks ahead from
  there. Surfaced by the new closed-loop `controller_benchmark` run.

## [0.5.0] - 2026-06-04

Theme: **making the experimental catalog usable.** v0.4.0 added the breadth of
classical planners; this release makes the catalog navigable and honest about
trade-offs. It adds a second reactive controller (ND) so the local side spans two
paradigms, two closed-loop comparison benchmarks (global planners and reactive
controllers) that measure rather than assert, a selection guide and onboarding so
users can find the right plugin, and puts the generative Mode B planner on the
same comparison footing as the classical ones. The closed-loop benchmark also
caught and fixed a real backward-lookahead bug that unit tests missed — evidence
that the measurement layer earns its place.

### Added

- **Generative Mode B in the planner comparison** — the planner benchmark now also
  runs `nav2_diffusion_global_planner::DiffusionGlobalPlanner` (its default analytic
  `FanPathModel`, no ONNX, deterministic) alongside the eight classical planners,
  putting *generative propose + classical dispose* on the same footing. It is the
  fastest after RRT-Connect and returns optimal paths on the clear and off-centre-gap
  scenarios, but reports **no path** on the slalom: the analytic fan cannot propose
  an S-shape, so the deterministic validity layer rejects every candidate and the
  planner fails closed — the intended behaviour of the propose/dispose split (a
  trained model would propose richer shapes; the same safety layer would still gate
  them). See `docs/planner_comparison.md`.
- **Planner / controller selection guide** — `docs/choosing_a_planner.md`: when to
  use each of the eight classical global planners, the two reactive controllers,
  and the generative options (with decision tables and Mermaid flowcharts), plus
  honest guidance on when upstream Nav2 (NavFn/Smac, MPPI/DWB/RPP) is the better
  choice. Links the offline comparison reports.
- **Reactive controller comparison benchmark** — `controller_benchmark`
  (`nav2_planner_benchmarks`) rolls out the VFH+ and ND controllers closed-loop
  against a live `Costmap2DROS` with a unicycle model on shared scenarios (open,
  frontal obstacle, off-centre corridor), recording goal success, path length,
  minimum clearance, steering smoothness, and corridor centring, and writes
  `docs/controller_comparison.md`. Both controllers clear the obstacles and reach
  the goal; they trade berth against smoothness with neither dominating.
- **Classical ND (Nearness Diagram) local controller** —
  `nav2_nd_controller::NDController`, a second reactive `nav2_core::Controller`
  (a different paradigm from VFH+). ND (Minguez & Montano, 2004) scores per-sector
  obstacle nearness, selects a navigable gap toward the goal, and adds a *safety
  deflection* that steers away from whichever side has a close obstacle — so the
  robot centres itself in corridors and squeezes through tight gaps, the
  behaviour that distinguishes ND from a histogram valley-cost method. Simplified
  ND (per-sector nearest distance + region/gap selection + symmetric deflection).
  Upstream Nav2 has neither ND nor VFH. Deterministic. Closed-loop gtests vs a
  live `Costmap2DROS` (clear path, frontal obstacle, **deflection away from a
  close right-side obstacle**, no plan, at goal), registered via pluginlib, added
  to CI and a bringup controller_server example.

### Changed

- **getting_started.md** now onboards the classical planners/controllers too:
  how to swap any of the eight global planners and the two reactive controllers
  via `planner_server` / `controller_server`, a plugin-class table pointing at the
  bringup example yamls, and links to the selection guide and comparison reports.

### Fixed

- **Lookahead carrot could point backwards** in `VFHController` and `NDController`:
  the lookahead picked the first plan pose at least `lookahead_distance` from the
  robot scanning from the plan start, so once the robot had advanced, already-
  passed poses behind it satisfied the distance first and the carrot flipped
  backwards — making the controller spin in place. Both now find the nearest plan
  pose and look ahead forward from there. Surfaced by the new
  `controller_benchmark` closed-loop rollout (single-call unit tests did not catch
  it).

## [0.4.0] - 2026-06-04

Theme: **classical planners Nav2 lacks.** The repo was renamed
`nav2_diffusion_planner` -> `nav2_experimental_planner` to reflect a broader
charter: host experimental planners not in upstream Nav2, generative *and*
classical. This release adds eight classical `nav2_core::GlobalPlanner` families
spanning every major paradigm (sampling, incremental, grid-A* speed-up,
any-angle, anytime, geometric), the first classical `nav2_core::Controller`
(VFH+), and an offline comparison benchmark across all eight global planners.
"Learned models propose. Classical safety disposes. Nav2 executes" — now with a
classical bench deep enough to be the disposer.

### Added

- **Classical VFH+ local controller** — `nav2_vfh_controller::VFHController`, a
  reactive Vector Field Histogram Plus `nav2_core::Controller` in the new
  `nav2_vfh_controller` package — the repo's first classical Controller (Mode A),
  alongside the generative `nav2_diffusion_controller`. VFH+ (Ulrich &
  Borenstein, 1998) builds a polar histogram of obstacles around the robot (each
  enlarged by the robot radius so a free sector is traversable), reduces it to
  free/blocked angular sectors, and steers toward the free sector best balancing
  goal heading, turning effort, and steering smoothness — a cheap reactive method
  with no trajectory rollout. Upstream Nav2 ships only optimisation-based local
  controllers (DWB, MPPI, Regulated Pure Pursuit); it has no VFH. Deterministic.
  Closed-loop gtests vs a live `Costmap2DROS` with a static TF (drives straight
  on a clear path, steers around an obstacle dead ahead while still advancing,
  stops with no plan, stops at the goal), registered via pluginlib, added to CI
  and a bringup controller_server example.
- **Classical planner comparison benchmark** — `nav2_planner_benchmarks`
  (`planner_benchmark` executable) loads all eight classical
  `nav2_core::GlobalPlanner` plugins via pluginlib and runs them on shared
  scenarios (clear, off-centre gap, slalom) against a live `Costmap2DROS`,
  recording success, path length, pose count, and median plan time, and writes
  `docs/planner_comparison.md`. Shows the trade-offs at a glance: RRT-Connect is
  fastest but its paths are longest; Lazy Theta* / visibility graph give the
  shortest (any-angle) paths; D* Lite's warm incremental replans are cheap; RRT*
  is near-optimal but slow to converge.
- **Classical visibility-graph global planner** —
  `nav2_visibility_graph_planner::VisibilityGraphPlanner`, a continuous-space
  geometric `nav2_core::GlobalPlanner` in the new `nav2_visibility_graph_planner`
  package. It reasons about obstacle geometry rather than grid cells: graph
  vertices are obstacle convex corners (extracted from the costmap) plus the start
  and goal, edges connect mutually visible vertices (line-of-sight), and A* over
  the graph returns a piecewise-straight corner-hugging route. The only
  geometry-based (non-grid, non-sampling) planner in the repo; upstream Nav2 has
  none. All-pairs visibility is O(V^2), capped by `max_corners` (warns when
  truncated). Fully deterministic. Closed-loop gtests vs a live `Costmap2DROS`
  (straight line to an off-axis goal on a clear map, route through an off-centre
  gap, solid wall, occupied goal, cancel), registered via pluginlib, added to CI
  and a bringup planner_server example.
- **Classical ARA\* anytime global planner** —
  `nav2_ara_star_planner::ARAStarPlanner`, an anytime / bounded-suboptimal
  `nav2_core::GlobalPlanner` in the new `nav2_ara_star_planner` package. ARA*
  (Likhachev, Gordon and Thrun, 2003) runs weighted-A* searches with a shrinking
  inflation factor epsilon: the first search returns a path within epsilon of
  optimal cheaply, and later searches reuse the prior effort (OPEN + INCONS) to
  tighten the bound toward 1 (optimal) within a per-plan expansion budget — so it
  returns a valid path fast and keeps improving. Upstream Nav2 has no anytime /
  bounded-suboptimal planner; this adds that capability. Fully deterministic.
  Closed-loop gtests vs a live `Costmap2DROS` (clear map, **inflated-epsilon
  configuration exercising the improvement loop**, off-centre gap, solid wall,
  occupied goal, cancel), registered via pluginlib, added to CI and a bringup
  planner_server example.
- **Classical Lazy Theta\* any-angle global planner** —
  `nav2_lazy_theta_star_planner::LazyThetaStarPlanner`, an any-angle
  `nav2_core::GlobalPlanner` in the new `nav2_lazy_theta_star_planner` package.
  Theta* lets a node's parent be any earlier node it has line of sight to, so
  paths bend only at obstacle corners instead of following the 8 grid directions;
  the lazy variant (Nash, Koenig and Tovey, 2010) defers the line-of-sight check
  to roughly one per expanded node. Upstream Nav2 ships eager Theta* in
  nav2_theta_star_planner; this adds the distinct lazy variant. Line-of-sight is
  sampled at the same resolution the path is densified at, so every accepted
  segment stays collision-free. Treats the costmap as a binary free/blocked grid
  (cells >= `lethal_threshold` are obstacles). Fully deterministic. Closed-loop
  gtests vs a live `Costmap2DROS` (clear map, **single straight any-angle line to
  an off-axis goal**, off-centre gap, solid wall, occupied goal, cancel),
  registered via pluginlib, added to CI and a bringup planner_server example.
- **Classical Jump Point Search (JPS) global planner** —
  `nav2_jps_planner::JPSPlanner`, an optimal grid-A* speed-up
  `nav2_core::GlobalPlanner` in the new `nav2_jps_planner` package. JPS (Harabor &
  Grastien, 2011) exploits grid path symmetry to jump over chains of cells A*
  would expand individually, pushing only jump points (turning / forced-neighbour
  cells) onto the open list — same optimal 8-connected path, far fewer
  expansions. Upstream Nav2's grid planners (NavFn, Smac) do not include JPS.
  Treats the costmap as a binary free/blocked grid (cells >= `lethal_threshold`
  are obstacles), so it ignores graded inflation costs. Fully deterministic.
  Closed-loop gtests vs a live `Costmap2DROS` (clear map, off-centre gap, solid
  wall, occupied goal, cancel), registered via pluginlib, added to CI and a
  bringup planner_server example.
- **Classical D\* Lite incremental global planner** —
  `nav2_dstar_lite_planner::DStarLitePlanner`, an incremental-search
  `nav2_core::GlobalPlanner` in the new `nav2_dstar_lite_planner` package.
  Upstream Nav2's global planners (NavFn, Smac, Theta*) replan from scratch every
  cycle; D* Lite (Koenig & Likhachev, 2002) searches the 8-connected costmap grid
  backward from the goal, caches g/rhs values across `createPlan` calls, and on
  the next plan shifts the priority keys for the moved robot (`km`) and repairs
  only the vertices whose costs changed since the last snapshot — far cheaper when
  the costmap changes little between cycles. Adds the incremental family Nav2
  lacks. Fully deterministic. Closed-loop gtests vs a live `Costmap2DROS` (clear
  map, off-centre gap, **incremental replan that reuses cached state to avoid a
  newly dropped wall**, solid wall, occupied goal, cancel), registered via
  pluginlib, added to CI and a bringup planner_server example.
- **Classical PRM global planner** — `nav2_prm_planner::PRMPlanner`, a
  Probabilistic Roadmap `nav2_core::GlobalPlanner` in the new `nav2_prm_planner`
  package. Samples collision-free milestones over the global costmap, wires
  nearby ones (k-nearest within a radius, collision-free edges) into an
  undirected graph, splices in the start and goal, and returns the shortest path
  by Dijkstra search. Adds the roadmap family Nav2's search-only global planners
  lack. Deterministic for a fixed `random_seed`. Closed-loop gtests vs a live
  `Costmap2DROS` (clear map, off-centre gap, solid wall, occupied goal, cancel),
  registered via pluginlib, added to CI and a bringup planner_server example.
- **Classical RRT-Connect global planner** — `nav2_rrt_planner::RRTConnectPlanner`,
  a bidirectional sampling-based `nav2_core::GlobalPlanner` (second planner in the
  `nav2_rrt_planner` package). Grows trees from both the start and the goal and
  greedily connects them, threading narrow passages in far fewer iterations than
  plain RRT/RRT* (feasible, not asymptotically optimal — use RRTStarPlanner when
  shortest-path matters). Still absent from upstream Nav2's search-only global
  planners. Deterministic for a fixed `random_seed`. Closed-loop gtests vs a live
  `Costmap2DROS` (clear map, off-centre gap, solid wall, occupied goal, cancel),
  registered via pluginlib, added to CI and the bringup planner_server example.
- **Classical RRT\* global planner** — `nav2_rrt_planner::RRTStarPlanner`, a
  sampling-based `nav2_core::GlobalPlanner`. Upstream Nav2 ships only search-based
  global planners (NavFn, Smac, Theta*); this adds the missing sampling family
  (goal-biased RRT* with neighbourhood rewiring over the global costmap),
  deterministic for a fixed `random_seed`. It routes through gaps/around walls
  the grid neighbourhoods miss. Closed-loop gtests vs a live `Costmap2DROS`
  (clear map, off-centre gap, solid wall, occupied goal, cancel) + a bringup
  planner_server example. First non-generative planner in the renamed
  nav2_experimental_planner repo (planners not in upstream Nav2).

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
- **Mode B demo GIF** (`docs/mode_b_demo.gif`, `tools/mode_b_demo.py`) rendered
  from the shipped generative `PathFlowPlanner`: K global-path proposals, the
  costmap rejecting colliding candidates (red) and selecting the shortest safe
  path (green) as the obstacle sweeps left/right.
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

[0.9.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.9.0
[0.8.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.8.0
[0.7.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.7.0
[0.6.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.6.0
[0.5.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.5.0
[0.4.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.4.0
[0.3.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.3.0
[0.2.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.2.0
[0.1.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.1.0
