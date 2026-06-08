# Nav2PlannerBattle ⚔️

[![CI](https://github.com/rsasaki0109/Nav2PlannerBattle/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/Nav2PlannerBattle/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Play Battle](https://img.shields.io/badge/▶_Play-Battle_online-ffd34d?style=for-the-badge)](https://rsasaki0109.github.io/Nav2PlannerBattle/)

**Real Nav2 planners & controllers, head-to-head — browser + benchmarks.**

> Learned models propose. Classical safety disposes. Nav2 executes.

<p align="center">
  <a href="https://rsasaki0109.github.io/Nav2PlannerBattle/"><img src="docs/battle_race.gif" width="640" alt="Planner Battle — Mode A race"></a>
</p>

<p align="center"><strong><a href="https://rsasaki0109.github.io/Nav2PlannerBattle/">▶ Play online</a></strong> · no install · real <code>battle_trace</code> plugins</p>

<p align="center">
  <img src="docs/battle_maze.gif" width="310" alt="Micro-mouse maze">
  &nbsp;
  <img src="docs/battle_duel.gif" width="310" alt="Planner duel">
</p>

---

## Planner Battle

| Mode | What | Play |
|---|---|---|
| **🏁 Race** | Controllers race to goal (VFH+, ND, learned, threading, …) | [frontal](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=A&s=1) · [maze](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=A&s=4) |
| **🧭 Duel** | Global planners draw paths (RRT*, JPS, Diffusion, omni/diff/Ackermann, …) | [slalom](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=B&s=6) · [kinematics gap](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=B&s=3) · [hard maze](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=B&s=9) |
| **🏆 Championship** | Points across all scenarios | [Race](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=C) · [Duel](https://rsasaki0109.github.io/Nav2PlannerBattle/?m=C&sub=B) |

Real `nav2_core` plugins — no scripted winners. Traces: `ros2 run nav2_planner_benchmarks battle_trace` · GIFs: [`tools/record_battle_gif.py`](tools/record_battle_gif.py)

---

## What is this?

Nav2 extension — **not** a replacement. Generative models propose trajectories/paths; a deterministic safety layer validates; Nav2 executes via standard plugins.

| | |
|---|---|
| **Adds** | 8 classical GlobalPlanners, VFH+/ND controllers, generative Mode A/B models |
| **Scope** | AMR / warehouse / delivery |
| **Compare** | [planners](docs/planner_comparison.md) · [controllers](docs/controller_comparison.md) · [models](docs/model_comparison.md) |

Details: [architecture](docs/architecture.md) · [safety](docs/safety.md) · [getting started](docs/getting_started.md)

---

## Architecture

```mermaid
flowchart LR
  G["Goal"] --> OB["Costmap + odom"]
  OB --> M["TrajectoryModel"]
  M --> SG["Safety gate"]
  SG --> CMD["cmd_vel"]
  SG -->|fail| FB["MPPI / RPP / stop"]
  FB --> CMD
```

Rules: models never publish `cmd_vel` directly · no Nav2 fork · GPU failure → stop/fallback. Full spec: [architecture.md](docs/architecture.md) · [safety.md](docs/safety.md)

---

## Demos

<p align="center">
  <img src="docs/demo.gif" width="400" alt="Mode A candidates">
  &nbsp;
  <img src="docs/mode_b_demo.gif" width="300" alt="Mode B path selection">
</p>

| Demo | Script |
|---|---|
| Costmap-conditioned flow | [`tools/costmap_demo.py`](tools/costmap_demo.py) |
| Mode B propose→validate | [`tools/mode_b_demo.py`](tools/mode_b_demo.py) |
| Gazebo courses | [`nav2_diffusion_sim`](generative/nav2_diffusion_sim) · [`docs/simulation.md`](docs/simulation.md) |

Mode B global planner: [nav2_diffusion_global_planner](generative/nav2_diffusion_global_planner) · limits & ceilings: [generative_limits.md](docs/generative_limits.md)

---

## Docs

| Topic | Link |
|---|---|
| Training | [docs/training.md](docs/training.md) |
| Benchmarks | [docs/benchmarking.md](docs/benchmarking.md) |
| Model zoo | [docs/model_zoo.md](docs/model_zoo.md) |
| Visualization | [docs/visualization.md](docs/visualization.md) |
| Roadmap | [docs/roadmap.md](docs/roadmap.md) · [CHANGELOG.md](CHANGELOG.md) |
| Pick a planner | [docs/choosing_a_planner.md](docs/choosing_a_planner.md) |

---

## Status

**v0.11.0** — Mode B transformer threads gaps; MCAP/Foxglove viz; Gazebo courses. API unstable until 1.0.0.

> ⚠️ Not safety-certified. Real hardware needs E-stop, speed limits, ODD, on-site risk assessment. [safety.md](docs/safety.md)
