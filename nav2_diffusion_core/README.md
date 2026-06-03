# nav2_diffusion_core

ROS 非依存に近い trajectory schema, scoring concept, shared utilities。

**Status: 最小実装あり（ビルド & テスト通過）。**

生成モデル・Nav2・安全層が共有する、ROS にできるだけ依存しないコアロジックを置く。ユニットテストしやすく、runtime / training の両方から参照できることを狙う。

## 現状の実装

- `nav2_diffusion_core/trajectory.hpp`: `TrajectoryPoint`（time-indexed SE(2) サンプル）と `Trajectory`（pose 列 + `model_score`）
- `pathLength()` / `duration()` ユーティリティ
- `nav2_diffusion_core/scoring.hpp`: Trajectory Scorer（§3.3）。`endpointDistance()` / `totalTurning()` / `scoreTrajectory()`（goal 接近 + smoothness の soft score）。安全は別層で担保し、これは soft preference のみ。
- `nav2_diffusion_core/trajectory_model.hpp`: 生成モデルの抽象 `TrajectoryModel` と `ModelContext`（§5.2 Generative Model Plugin の seam）。学習モデル（PyTorch/ONNX/TensorRT）はこの interface の裏に差し込む。
- `nav2_diffusion_core/fan_rollout_model.hpp`: 組み込みの解析モデル `FanRolloutModel`（角速度ファンを rollout する multimodal プレースホルダ）。学習モデルが入るまでパイプライン全体を動かすためのもの。
- gtest（`test/test_trajectory.cpp`, `test/test_scoring.cpp`, `test/test_fan_rollout_model.cpp`）

## 想定する責務

- Trajectory candidate のコア表現と変換（SE(2) pose 列 ⇔ control sequence）
- Kinematic Projector（diff / omni / ackermann への投影）— [../docs/architecture.md](../docs/architecture.md) §3.3
- Trajectory Scorer の概念（progress / smoothness / clearance / path alignment）— architecture §3.3
- 共有 utility（座標変換ヘルパ、時刻同期、正規化）

## Dependency Policy

ROS 2, Nav2, lightweight C++ dependencies のみ（[../docs/architecture.md](../docs/architecture.md) §12.2）。重い Python 学習依存を持ち込まない。
