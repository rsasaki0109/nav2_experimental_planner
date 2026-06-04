# Changelog

All notable changes to this project are documented here. The project aims to
follow [Semantic Versioning](https://semver.org/); APIs are not yet stable
before 1.0.0 (see [docs/roadmap.md](docs/roadmap.md)).

## [Unreleased]

### Added
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

[0.8.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.8.0
[0.7.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.7.0
[0.6.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.6.0
[0.5.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.5.0
[0.4.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.4.0
[0.3.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.3.0
[0.2.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.2.0
[0.1.0]: https://github.com/rsasaki0109/nav2_experimental_planner/releases/tag/v0.1.0
