# Training Architecture

> 関連: [architecture.md](architecture.md) §7 Inference / §12.2 Runtime/Training Separation、[simulation.md](simulation.md)、[../nav2_diffusion_training/README.md](../generative/nav2_diffusion_training/README.md)

## データセット生成（実装済み）

`nav2_diffusion_training`（ament_python）で、以下 3 経路から学習サンプル（base frame の未来軌道ラベル、§6.3）を生成できる。いずれも `build_samples()` に集約され、`save_jsonl()` で書き出す。

```python
from nav2_diffusion_training import build_samples, save_jsonl, TrackState, unicycle_to_goal
from nav2_diffusion_training.rosbag_io import track_from_bag

# 1) 実機/sim の rosbag から（§6.2）
track = track_from_bag('my_run_bag', topic='/odom')

# 2) rule-based expert から sim 無しで合成（§6.5）
track = unicycle_to_goal(TrackState(0.0, 0.0, 0.0, 0.0), goal_x=3.0, goal_y=1.0)

# 3) 任意の TrackState 列から
track = [TrackState(time=t * 0.1, x=t * 0.03, y=0.0, yaw=0.0) for t in range(100)]

samples = build_samples(track, history=4, horizon=20)
save_jsonl(samples, 'dataset.jsonl')
```

各サンプルは `observation_window`（過去の絶対 pose 列）と `action_label`（anchor の base frame に変換した未来軌道）を持ち、[architecture.md](architecture.md) §4.4 の SE(2) 表現に一致する。学習モデルはこのデータで `nav2_diffusion_core::TrajectoryModel`（§5.2 seam）の裏に実装する。

> 注: 学習ループ本体（PyTorch 等）と実推論バックエンド（ONNX/TensorRT）は依存が重いため、本パッケージの dataset 層とは分離して追加する（§12.2）。

## 6.1 Training Pipeline Overview

学習パイプラインは、runtime から分離する。runtime package に巨大な Python 学習依存を持ち込まない。

| Stage | 内容 |
|---|---|
| Data Collection | simulation, rosbag, teleop, expert planner から収集 |
| Data Normalization | TF 変換、時刻同期、sensor crop、frame 統一 |
| Label Generation | future trajectory, expert action, collision label, progress label |
| Dataset Curation | unsafe, duplicate, corrupted, low-quality sample の除外 |
| Training | diffusion / flow / transformer 等の学習 |
| Open-loop Eval | ADE/FDE だけでなく collision 予測、goal progress、smoothness を評価 |
| Closed-loop Sim Eval | Gazebo / Isaac Sim / scenario suite で評価 |
| Safety Qualification | safety gate rejection、fallback 率、latency を検証 |
| Model Registry | model card, manifest, benchmark hash を保存 |
| Deployment Export | ONNX / TensorRT / quantized model へ変換 |

## 6.2 Data Sources

| Source | 用途 | 優先度 |
|---|---|---|
| Nav2 Expert in Simulation | Smac + MPPI / RPP で大量軌道を生成 | High |
| Gazebo | 軽量 CI、Nav2 標準 demo、headless regression | High |
| Isaac Sim | warehouse、sensor realism、RGB/depth、domain randomization | Medium-High |
| Recorded Bags | 実機分布、sensor noise、latency、現場特有の障害物 | High |
| Human Teleoperation | 社会的・暗黙知・リカバリ行動 | High |
| Intervention Logs | 自律失敗時の人間修正 | High |
| Synthetic Dynamic Obstacles | 人混み、台車、フォークリフト、交差 | Medium |
| Camera Datasets | visual extension 用 | Medium |

Isaac Sim は ROS 2 navigation tutorial や ROS 2 bridge を持ち、warehouse scenario や Nav2 連携に使えるため、v0.5 以降の sim-to-real・視覚入力・センサ多様化に向いている。

## 6.3 Dataset Schema

データセットは「ROS bag そのもの」ではなく、学習向けに正規化した trajectory dataset として管理する。

| Element | 内容 |
|---|---|
| observation_window | 過去数フレームの costmap / scan / odom / optional image |
| path_context | global path snippet, local goal |
| robot_state | pose, velocity, acceleration estimate |
| action_label | future trajectory or velocity sequence |
| safety_label | collision, near-collision, footprint violation |
| scene_metadata | map type, dynamic obstacle count, robot type |
| timing_metadata | sensor latency, control period, dropped frame |
| source_metadata | sim / real / teleop / expert planner |
| split_metadata | train / val / test / hidden benchmark |

## 6.4 Training Objectives

v0.1 の推奨 objective は、複雑にしすぎない。

| Objective | 目的 |
|---|---|
| trajectory imitation | expert / teleop 軌道に近づける |
| denoising / score matching | diffusion planner の主目的 |
| goal progress | ゴールへ進む行動を促す |
| collision awareness | unsafe future を避ける prior |
| smoothness | jerk, oscillation 低減 |
| path consistency | global path intent を尊重 |
| diversity | 複数候補の mode collapse 防止 |
| uncertainty calibration | OOD / confidence gating に使う |

重要なのは、collision loss で安全を「学習だけに任せない」こと。学習は prior であり、runtime 安全は deterministic safety layer で担保する（[safety.md](safety.md) 参照）。

## 6.5 Expert Generation Strategy

v0.1 では、人間データだけに依存しない。以下を混ぜる。

| Expert | 長所 | 短所 |
|---|---|---|
| Smac + MPPI | Nav2 native、再現可能、大量生成可能 | 社会的行動は限定的 |
| MPPI with tuned critics | 動的障害物に比較的強い | tuning 依存 |
| Human Teleop | 暗黙知、譲り合い、狭路突破 | データ収集コスト |
| Recovery demonstrations | stuck 脱出に有効 | rare data |
| Rule-based social planner | 初期 human-aware prior | 現実の複雑さには弱い |
| RL expert in sim | 多様な回避行動 | sim-to-real gap |

## 6.6 Sim-to-Real Strategy

sim-to-real は追加機能ではなく、初期から設計に入れる。

1. **Costmap-first representation** — カメラ画像に過度依存せず、Nav2 local costmap を主条件にする。
2. **Sensor degradation training** — Laser dropout、pointcloud noise、遅延、costmap flicker を学習時に入れる。
3. **Dynamics randomization** — 最大速度、加速度、摩擦、制動遅れ、回転半径を randomize する。
4. **Latency randomization** — 推論遅延・sensor timestamp ずれを含める。
5. **Shadow Mode validation** — 実機で `cmd_vel` を出す前に、既存 Nav2 走行中に裏で候補生成と安全判定を記録する。
6. **Field intervention loop** — fallback や人間介入が発生した場面を優先的に再学習データへ戻す。
