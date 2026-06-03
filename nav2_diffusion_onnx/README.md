# nav2_diffusion_onnx

ONNX Runtime 推論バックエンド（optional）。

**Status: 実装あり（onnxruntime があればビルド & テスト通過。無ければ空パッケージとして graceful skip）。**

`nav2_diffusion_core::TrajectoryModel`（§5.2 seam / §7.2 Runtime Backend）の ONNX 実装。学習済みモデルを controller の生成段に差し込むための実バックエンド。

## 内容

- `OnnxTrajectoryModel`: ONNX モデルを読み込み、context ベクトル `[goal_x, goal_y, linear_speed, max_angular_speed]` を入力に `[1, K, H, 3]`（K 候補 × H ステップ × x/y/yaw, base frame）を推論し、`Trajectory` 候補へ変換する。
- `scripts/make_test_model.py`: 検証用の小さな ONNX モデル（PyTorch で線形層を export）を生成。テスト時に build ディレクトリへ出力（git には含めない）。
- gtest（`test/test_onnx_trajectory_model.cpp`）: 実モデルをロード→推論→候補生成、決定論性を検証。

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
| output `trajectories` | float `[1, K, H, 3]` = K 候補 × H ステップ × (x, y, yaw)、base frame |

各ステップの時刻は `time_step`（既定 0.1s）× index。学習側はこの契約に合わせて export する。
