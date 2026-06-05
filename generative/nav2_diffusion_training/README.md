# nav2_diffusion_training

dataset, training, export pipeline。

**Status: dataset builder あり（ament_python / pytest 通過）。runtime から分離。**

学習パイプライン（[../docs/training.md](../docs/training.md)）。Python / dataset tools。**runtime package には持ち込まない**（[../docs/architecture.md](../docs/architecture.md) §12.2）。

## 現状の実装

- `nav2_diffusion_training.dataset`: 記録した SE(2) トラック（`/odom` 等）から学習サンプルを生成
  - `TrackState`（time, x, y, yaw）
  - `build_samples(track, history, horizon, stride)`: 各 anchor で **observation_window（過去の絶対 pose 列）** と **action_label（base frame に変換した未来軌道）** を生成（§6.3 schema / §4.4 SE(2)）
  - `save_jsonl(samples, path)`: JSON Lines 書き出し
  - stdlib のみ（依存軽量）
- `nav2_diffusion_training.rosbag_io`: `track_from_bag(bag_uri, topic='/odom')` — rosbag2 の odometry トピックを `TrackState` 列へ取り込む薄い adapter（§6.2）。`rosbag2_py` / `rclpy` serialization を使用。dataset 本体とは別モジュールなので、rosbag2_py 無しでも dataset は使える。
- `nav2_diffusion_training.experts`: sim 不要の rule-based expert（§6.5）。`unicycle_to_goal(start, goal_x, goal_y, ...)` が start→goal を unicycle 追従する expert track を生成。`build_samples` に渡せば**合成 imitation データセット**を sim 無しで作れる。
- `nav2_diffusion_training.train`: 最小の PyTorch 学習 + ONNX export（§6.4）。synthetic expert データで `TinyPlanner` を学習し、**C++ `OnnxTrajectoryModel` と同じ I/O 契約**（context `[1,4]` → trajectories `[1,K,H,3]`）で ONNX 出力。`train_and_export(path)` で一発。PyTorch は重い optional 依存のため `__init__` からは import せず、テストは torch 不在時に自動 skip（CI など）。
- pytest（`test/test_dataset.py`, `test/test_rosbag_io.py`, `test/test_experts.py`, `test/test_train.py`）+ ament lint（copyright / flake8 / pep257）

### 生成モデル4系統（OSS ギャップを埋める実装）

`nav2_diffusion_training.generative_planners` に、調査で「移動ロボット/Nav2 向け OSS が存在しない」と確認した4系統を実装（[docs リサーチ結果]、設計 §5.3）。いずれも context `[1,4]` → `[1,K,H,3]` の同一契約で ONNX export され、**C++ `OnnxTrajectoryModel` に無改造でロード可能**（flow モデルで実機 gtest 通過を確認）。

| 系統 | クラス | 概要 |
|---|---|---|
| flow matching | `FlowMatchingPlanner` | conditional flow matching、single/few-step ODE で K 候補を1〜数フォワード生成（GoalFlow 系、最速） |
| diffusion | `DiffusionPlanner` | cosine schedule の DDPM 学習 + DDIM サンプリング（JLAP 系） |
| consistency | `ConsistencyPlanner` | 1ステップ蒸留（noised→x0）で最速生成（CTM 系） |
| transformer | `TransformerPlanner` | DETR 風 set-prediction。K 個の learned query token が cross-attention で各々軌道をデコード（noise sampling なしの決定論的 single-forward、多様性は query が担う） |

```python
from nav2_diffusion_training.generative_planners import train_and_export
train_and_export('flow', 'flow.onnx')        # or 'diffusion' / 'consistency' / 'transformer'
# → controller の model_plugin: nav2_diffusion_onnx::OnnxTrajectoryModel, model_path: flow.onnx
```

ONNX は重みインラインの単一ファイルで出力（配布容易）。

### costmap 条件付き（研究の本丸・OSS ギャップ）

`CostmapFlowPlanner` / `CostmapDiffusionPlanner` / `CostmapConsistencyPlanner` / `CostmapTransformerPlanner` と `train_and_export_costmap(path, kind=...)` は、**egocentric local costmap パッチ + goal** を条件に K 候補を生成し、**2 入力 ONNX**（`context [1,4]` + `costmap [1,1,S,S]` → `[1,K,H,3]`）を export する。controller 側は `costmap_patch_size:=S` でパッチを供給し、`OnnxTrajectoryModel` が自動で costmap 入力を検出して渡す。合成データは「障害物が左→右へ回避」のように costmap と expert を相関させる。transformer は costmap パッチを strided conv でトークン化し、context トークンと併せた memory に query token が cross-attention する。

```python
from nav2_diffusion_training.generative_planners import train_and_export_costmap
train_and_export_costmap('costmap_flow.onnx', kind='flow')         # or 'diffusion' / 'consistency' / 'transformer'
```

### 学習↔推論の一周（検証済み）

```
expert (unicycle_to_goal) → build_samples → train_and_export(ONNX)
  → nav2_diffusion_onnx::OnnxTrajectoryModel が同じ契約でロード → controller の生成段
```

`test_train.py` は train → ONNX export → onnxruntime ロードで出力形状が契約（`[1,K,H,3]`）に一致することを検証する。

```python
from nav2_diffusion_training.rosbag_io import track_from_bag
from nav2_diffusion_training import build_samples, save_jsonl

track = track_from_bag('my_run_bag', topic='/odom')
save_jsonl(build_samples(track, history=4, horizon=20), 'dataset.jsonl')
```

## 想定する内容

- Data Collection / Normalization / Label Generation / Curation（training §6.1）
- Dataset Schema 実装（training §6.3）
- Training objectives（training §6.4）
- Open-loop / Closed-loop eval（training §6.1）
- Deployment Export（ONNX / TensorRT / quantized — [../docs/deployment.md](../docs/deployment.md)）

依存は pip / container / optional とし、runtime とは別管理にする。
