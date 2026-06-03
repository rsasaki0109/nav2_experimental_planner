# Roadmap

> 関連: [architecture.md](architecture.md)、[risks.md](risks.md)

## v0.1 — Nav2 Native MVP

**Theme: Costmap-conditioned generative local controller**

| Area | Deliverable |
|---|---|
| Nav2 Integration | Controller Plugin mode |
| Model | lightweight diffusion trajectory generator |
| Inputs | Goal, Global Path snippet, Local Costmap, Odometry |
| Outputs | trajectory candidates, best trajectory, `cmd_vel` |
| Safety | hard collision gate, velocity limits, stop fallback |
| Benchmark | Gazebo static / narrow / dynamic crossing |
| Baseline | MPPI and RPP comparison |
| Visualization | RViz candidate trajectories |
| Docs | Getting Started, Architecture, Safety MVP |
| Packaging | source build + Docker dev environment |

**Definition of Done:** 既存 Nav2 demo robot で、Controller 差し替えにより走行でき、MPPI / RPP と同一 scenario で比較でき、model failure 時に安全停止できる。

---

## v0.5 — Reproducible Learning & Hybrid Planning

**Theme: 研究から OSS 基盤へ**

| Area | Deliverable |
|---|---|
| Training | rosbag ingestion, simulation expert generation |
| Runtime | ONNX Runtime backend |
| Safety | fallback to MPPI/RPP, safety state diagnostics |
| Benchmark | dynamic obstacles, warehouse, U-trap |
| Simulation | Isaac Sim optional integration |
| Model Registry | model manifest + model card |
| Deployment | Shadow mode on real robot |
| Docs | Train your own model guide |
| Community | contribution guide, issue templates, RFC process |

**Definition of Done:** 第三者が自分の bag または sim data からモデルを学習し、benchmark に通し、shadow mode で評価できる。

---

## v1.0 — Production-Ready OSS

**Theme: 実運用可能な Nav2 generative navigation framework**

| Area | Deliverable |
|---|---|
| API Stability | plugin contracts and message schemas stabilized |
| Runtime | TensorRT backend |
| Safety | full safety architecture, failure mode coverage |
| Benchmark | public reproducible benchmark suite |
| Model Zoo | at least 2 validated models |
| Platforms | x86 GPU and Jetson validated |
| Nav2 | Planner mode optional, Controller mode stable |
| CI | CPU CI + GPU nightly + simulation regression |
| Docs | deployment, tuning, troubleshooting |
| Release | semantic versioning, binary release plan |

**Definition of Done:** Nav2 ユーザーが既存 robot に導入し、benchmark と safety guide を使って採用可否を判断できる。

---

## v2.0 — Generative Navigation Platform

**Theme: Diffusion を超えた Nav2 向け生成ナビゲーション基盤**

| Area | Deliverable |
|---|---|
| Model Families | diffusion, flow, consistency, transformer planner |
| World Models | optional predictive rollout planner |
| Human-aware | tracked humans, social cost, crowd scenarios |
| Semantic Navigation | camera / BEV / semantic costmap integration |
| Fleet Learning | intervention mining, continual dataset updates |
| Benchmark | leaderboard, hidden seeds, real robot reports |
| Ecosystem | third-party model plugins |
| Governance | maintainer council, RFC-based evolution |

**Definition of Done:** 研究者は新モデルをこの枠組みに載せて Nav2 benchmark で比較でき、企業ユーザーは安全層と deployment guide 込みで実機評価できる。
