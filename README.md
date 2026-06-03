<p align="center">
  <img src="docs/demo.gif" width="640" alt="DiffusionController: multimodal candidates (best=green / safe=blue / rejected=red) navigating around an obstacle">
</p>

<p align="center"><em>実際のパイプライン出力: 生成モデルが multimodal 候補を提案し、footprint 安全層が障害物 inflation 帯（赤い領域）に入る候補（赤線）を棄却し、scorer が最良候補（緑）を選んでクリアランスを保ちつつ回避。</em></p>

# nav2_diffusion_planner

[![CI](https://github.com/rsasaki0109/nav2_diffusion_planner/actions/workflows/ci.yml/badge.svg)](https://github.com/rsasaki0109/nav2_diffusion_planner/actions/workflows/ci.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Release](https://img.shields.io/github/v/release/rsasaki0109/nav2_diffusion_planner)](https://github.com/rsasaki0109/nav2_diffusion_planner/releases)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)

**Nav2向け Generative Navigation Framework**

> Learned models propose. Classical safety disposes. Nav2 executes.
> （生成モデルは候補を出す。安全層は候補を落とす。Nav2 は実行する。）

`nav2_diffusion_planner` は Nav2 を置き換えるプロジェクトではありません。Nav2 の既存アーキテクチャ（Behavior Tree / Lifecycle Node / Planner・Controller Plugin / Costmap / Collision Monitor）を最大限活かしたうえで、その上に **Diffusion / Flow Matching / Consistency / Transformer / World Model 系の生成型ナビゲーションモデルを安全に接続する** ための OSS 基盤です。

- **Scope:** AMR / Delivery Robot / Warehouse Robot / Service Robot
- **Out of Scope:** Manipulation, MoveIt, Humanoid, Full VLA, Multi-Agent Planning（主目的としては扱わない）
- **Core Positioning:** Nav2 Native な Generative Navigation Framework

---

## なぜこの OSS か

Nav2 は ROS 2 移動ロボット開発の事実上の実用基盤であり、Smac Planner / Regulated Pure Pursuit / MPPI など強力なスタックを持ちます。一方で、手設計のコスト・局所最適化・ヒューリスティックに依存するため、人混み・狭路・動的障害物・社会的ナビゲーション・センサーノイズ・チューニング負荷といった領域で限界が出やすい領域があります。

本 OSS はこれを「Nav2 の代替」ではなく **「Nav2 の能力拡張」** として解きます。生成モデルが multimodal な未来軌道候補を提案し、決定論的な安全層が検証し、Nav2 が実行する構造です。

詳細は [docs/architecture.md](docs/architecture.md) の §1（Problem Statement）/ §2（Vision）/ §15（Why This OSS Can Win）を参照してください。

---

## 設計哲学

| 原則 | 内容 |
|---|---|
| Learned models propose | 生成モデルは候補軌道の生成器であり、安全判定器ではない |
| Classical safety disposes | Costmap / footprint / 速度制約 / Collision Monitor が候補を棄却する |
| Nav2 executes | 既存の運用基盤（BT / Lifecycle / Controller Server）が実行する |

### Non-Negotiable Architecture Rules

1. Neural model が直接 `cmd_vel` を publish してはならない（必ず Safety Gate と Command Extractor を通す）。
2. Nav2 を fork しない（Plugin / BT / Lifecycle / Costmap / Collision Monitor との統合で実現する）。
3. Costmap / TF / Odometry を runtime truth source として扱う。
4. すべての候補軌道は可視化・記録できる（RViz + rosbag で説明可能）。
5. GPU が死んでもロボットは安全に止まるか fallback する。
6. Camera は optional（AMR / warehouse / delivery は LiDAR + costmap 構成が多い）。
7. Model Zoo のモデルは benchmark 通過済みでなければならない。

---

## Final Architecture Position

`nav2_diffusion_planner` の正しい初期形は次です。

> **Nav2 Controller Plugin として動く、costmap-conditioned generative trajectory proposal framework。**

最終形は、Diffusion / Flow Matching / Consistency Models / Transformer Planners / World Models を統合できる、Nav2 Native な Generative Navigation Framework です。

最初に作るべきものは「SOTA モデル」ではなく、以下です。

- Nav2 に自然に入る plugin 構造
- Future Trajectory Candidates の共通表現
- Safety Gate
- MPPI / RPP fallback
- RViz 可視化 / rosbag replay
- benchmark suite
- model manifest / model card
- training pipeline
- Jetson deployment path

---

## アーキテクチャ図

生成モデルが提案し、決定論的安全層が検証し、Nav2 が実行するパイプライン（Controller Plugin / Mode A）:

```mermaid
flowchart LR
  G["Goal / Global Path"] --> OB["Observation<br/>(local costmap, odom, TF)"]
  OB --> M["TrajectoryModel (seam)<br/>FanRolloutModel → ONNX/PyTorch/TensorRT"]
  M -->|"K candidates"| IV["Input-validity gate<br/>(stale TF/odom/costmap)"]
  IV --> KN["Kinematic limits"]
  KN --> FP["Footprint collision<br/>(local costmap)"]
  FP --> SC["Scorer<br/>(progress + smoothness)"]
  SC -->|"best"| EX["Command extractor"] --> CMD["cmd_vel"]
  FP -->|"no safe candidate"| FB["Fallback<br/>(MPPI / RPP / stop)"]
  FB --> CMD
  M -.->|"candidates / SafetyState"| VIZ["RViz markers / rosbag"]
```

> **Learned models propose. Classical safety disposes. Nav2 executes.**
> 学習モデル（`TrajectoryModel` の裏）を差し替えても、安全層・scoring・fallback・可視化はそのまま再利用できます。

データ生成から実行までの一周（各段ユニットテスト済み）:

```mermaid
flowchart LR
  EX["Rule-based expert<br/>unicycle_to_goal"] --> DS["build_samples<br/>(base-frame future traj)"]
  BAG["rosbag /odom"] --> RB["track_from_bag"] --> DS
  DS --> TR["PyTorch train + ONNX export<br/>(train_and_export)"]
  TR --> MD["model.onnx"]
  MD --> OB["OnnxTrajectoryModel<br/>(pluginlib)"]
  OB --> CT["DiffusionController<br/>(model_plugin)"]
  CT --> RUN["safety gate → scoring → cmd_vel"]
```

## costmap 条件付き生成（OSS-gap 実装）

<p align="center">
  <img src="docs/costmap_demo.gif" width="480" alt="costmap-conditioned CostmapFlowPlanner: as the obstacle sweeps left/right, the generated candidates veer to the opposite side">
</p>

<p align="center"><em>出荷モデル <code>CostmapFlowPlanner</code>（flow matching + egocentric costmap エンコーダ）そのものの出力。障害物（赤）が左右に動くと、生成された候補軌道が反対側へ veer する＝ costmap に条件付いた回避を学習している。再現は <a href="tools/costmap_demo.py">tools/costmap_demo.py</a>。</em></p>

調査（papers＋既存 OSS の突き合わせ）の結果、Nav2 地上ロボット向けの **flow / diffusion / consistency の local planner で公開実装が無い** ことを確認し、3系統＋costmap 条件付けを OSS-gap 実装として収録している（[docs/model_zoo.md](docs/model_zoo.md)）。6系統を同一シナリオで比較したオフライン leaderboard は [docs/model_comparison.md](docs/model_comparison.md)（`tools/benchmark_models.py` で再現可能）。

## ドキュメント地図

| ドキュメント | 内容 |
|---|---|
| [docs/architecture.md](docs/architecture.md) | コアアーキテクチャ（Problem / Vision / 5層構成 / Data Flow / Plugin / Inference / Repo 構造 / Why Win） |
| [docs/safety.md](docs/safety.md) | Safety Architecture（安全層 / 状態機械 / fallback / 安全 deliverables） |
| [docs/training.md](docs/training.md) | Training Architecture（データ収集 / dataset schema / objective / sim-to-real） |
| [docs/benchmarking.md](docs/benchmarking.md) | Benchmark Suite（baseline / scenario / metrics / leaderboard） |
| [docs/simulation.md](docs/simulation.md) | Simulation Strategy（Gazebo / Isaac Sim / golden scenarios） |
| [docs/deployment.md](docs/deployment.md) | Deployment Strategy（platform / Jetson / packaging / staged rollout） |
| [docs/roadmap.md](docs/roadmap.md) | Roadmap（v0.1 / v0.5 / v1.0 / v2.0） |
| [docs/risks.md](docs/risks.md) | Risks（technical / OSS operation / safety・liability） |
| [docs/getting_started.md](docs/getting_started.md) | Nav2 ユーザー向け導入（Controller 差し替え / demo） |
| [docs/contributing.md](docs/contributing.md) | 貢献ガイド（plugin / model / benchmark 追加） |
| [docs/model_zoo.md](docs/model_zoo.md) | Model Zoo（model card / manifest 一覧） |
| [docs/model_comparison.md](docs/model_comparison.md) | 6生成系のオフライン比較 leaderboard（`tools/benchmark_models.py` 自動生成） |

---

## Status

**v0.3.0** — Nav2 Controller Plugin（generative local controller、Mode A）+ **Nav2 GlobalPlanner Plugin（generative global planner、Mode B）** + 決定論的安全層 + ONNX バックエンド + **3生成系（flow / diffusion / consistency）** + **costmap+goal 条件付け** + 学習パイプライン + RViz 可視化 + benchmark suite。変更履歴は [CHANGELOG.md](CHANGELOG.md)、今後の計画は [docs/roadmap.md](docs/roadmap.md)。API は 1.0.0 まで安定化されていません。

> ⚠️ この OSS は安全認証済み製品ではありません。実機導入者は hardware EStop、速度制限、ODD（Operational Design Domain）定義、現場 risk assessment を必ず行ってください。詳細は [docs/safety.md](docs/safety.md) を参照。
