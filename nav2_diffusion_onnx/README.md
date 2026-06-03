# nav2_diffusion_onnx

ONNX Runtime 推論バックエンド（optional）。

**Status: 実装あり（onnxruntime があればビルド & テスト通過。無ければ空パッケージとして graceful skip）。**

`nav2_diffusion_core::TrajectoryModel`（§5.2 seam / §7.2 Runtime Backend、Mode A）と `nav2_diffusion_core::PathModel`（Mode B）の ONNX 実装。学習済みモデルを controller / global planner の生成段に差し込むための実バックエンド。

## 内容

- `OnnxTrajectoryModel`（Mode A）: ONNX モデルを読み込み、context ベクトル `[goal_x, goal_y, linear_speed, max_angular_speed]` を入力に `[1, K, H, 3]`（K 候補 × H ステップ × x/y/yaw, base frame）を推論し、`Trajectory` 候補へ変換する。
- `OnnxPathModel`（Mode B）: 生成型 GlobalPlanner 用。goal-aligned context `[goal_distance, 0]` を入力に `[1, K, H, 2]`（K 候補パス × H waypoint × x/y, goal-aligned frame）を推論し、goal bearing で回転・start で平行移動して map frame の `PathCandidate` へ変換、端点を start/goal にスナップする。`DiffusionGlobalPlanner` が `model_plugin` で pluginlib ロードする。
- `scripts/make_test_model.py` / `make_costmap_model.py` / `make_path_model.py`: 検証用の小さな ONNX モデルを生成（テスト時に build ディレクトリへ出力、git には含めない）。
- gtest（`test/test_onnx_trajectory_model.cpp`, `test/test_onnx_path_model.cpp`）: 実モデルをロード→推論→候補生成、決定論性・frame 変換・端点アンカーを検証。

## ビルド要件

onnxruntime を CMake から発見できること（いずれか）:

- `onnxruntime_vendor`（ROS）
- システムインストール
- `--cmake-args -Donnxruntime_DIR=/path/to/lib/cmake/onnxruntime`

発見できない場合、本パッケージは**空パッケージとしてビルドされ**（backend 無し）、ワークスペース全体の `colcon build` は壊れない。

```bash
colcon build --packages-select nav2_diffusion_onnx \
  --cmake-args -Donnxruntime_DIR=/path/to/onnxruntime/lib/cmake/onnxruntime
```

## モデル I/O 契約

| 項目 | 形 |
|---|---|
| input `context` | float `[1, 4]` = `[goal_x, goal_y, linear_speed, max_angular_speed]`（base frame の local goal） |
| input `costmap`（任意） | float `[1, 1, S, S]` = ロボット中心の egocentric local costmap パッチ（正規化 [0,1]）。モデルがこの名前の入力を持つ場合のみ controller が `costmap_patch_size:=S` で供給する |
| output `trajectories` | float `[1, K, H, 3]` = K 候補 × H ステップ × (x, y, yaw)、base frame |

`costmap` 入力の有無は backend が自動検出する（無ければ context のみで動作＝後方互換）。costmap 条件付きモデルは `nav2_diffusion_training.generative_planners.CostmapFlowPlanner` で学習・export できる。

各ステップの時刻は `time_step`（既定 0.1s）× index。学習側はこの契約に合わせて export する。

### PathModel I/O 契約（Mode B、`OnnxPathModel`）

| 項目 | 形 |
|---|---|
| input `context` | float `[1, 2]` = `[goal_distance, 0]`（goal-aligned frame。goal は `(d, 0)`） |
| output `paths` | float `[1, K, H, 2]` = K 候補パス × H waypoint × (x, y)、goal-aligned frame |

backend が start 姿勢と goal bearing で map frame に戻し、端点を start/goal にスナップする。学習・export は `nav2_diffusion_training.path_planners.train_and_export_path`（flow matching、多峰な detour を提案）。
