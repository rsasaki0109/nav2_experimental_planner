# Roadmap

> 関連: [architecture.md](architecture.md)、[risks.md](risks.md)

## 実装状況（v0.3.0 時点）

以下は計画（後続セクション）に対する現状サマリ。一部は計画を先取りして実装済み。

| 項目 | 計画 | 状況 |
|---|---|---|
| Nav2 Controller Plugin（Mode A） | v0.1 | ✅ 実装・closed-loop 統合テスト（in-process） |
| 安全層（入力検証/kinematic/footprint/状態機械/fallback） | v0.1–v1.0 | ✅ 実装 |
| multimodal 候補生成 + Trajectory Scorer | v0.1 | ✅ |
| TrajectoryModel plugin seam | v0.5/v2.0 | ✅ 先取り（pluginlib で実行時ロード） |
| ONNX Runtime backend | v0.5 | ✅ 先取り（optional パッケージ） |
| 生成モデル3系統（diffusion / flow / consistency） | v2.0 | ✅ 先取り（context-only + costmap 条件付き両方） |
| **costmap+goal 条件付き生成** | v0.1 theme | ✅（heading-aligned egocentric パッチ、ONNX 2入力） |
| 学習パイプライン（rosbag/expert→dataset→PyTorch→ONNX） | v0.5 | ✅ |
| RViz 可視化（候補/棄却理由/安全状態） | v0.1 | ✅ |
| benchmark suite（scenario/metrics/score/leaderboard/aggregate/runner） | v0.1–v1.0 | ✅ コア |
| MPPI/RPP fallback 委譲 | v0.5 | ✅ |
| Gazebo / loopback demo launch | v0.1 | ✅（launch・params。実走行 numbers は GPU/DDS 環境に依存） |
| OSS 運用（LICENSE/CI/docs/manifest/card/issue・PR・RFC/badge/release） | v0.5 | ✅ |
| 実機 shadow mode / 実走行 benchmark numbers | v0.5/v1.0 | ⬜ 実機・実sim 環境が必要 |
| TensorRT backend / Jetson 検証 | v1.0 | ⬜ |
| Planner Plugin（Mode B） | v1.0 | ✅ 先取り（**生成型 GlobalPlanner**: パス候補提案 → costmap 検証 → 最短安全パス選択。`PathModel` seam、analytic placeholder + 学習モデル差し替え可。生成型 Nav2 GlobalPlanner は OSS 不在を確認した上での実装） |
| social metrics / human-aware | v2.0 | ⬜ 人トラッキング log が必要 |
| world model planner | v2.0 | ⬜ 研究枠 |

> 注: 生成モデル・ONNX・costmap 条件付けなど v2.0 級の項目を先取りしている一方、実機/実 sim での数値収集（shadow mode・Gazebo benchmark）は実行環境依存で未着手。詳細は [CHANGELOG.md](../CHANGELOG.md)。

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
