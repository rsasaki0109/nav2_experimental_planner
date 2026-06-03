# nav2_diffusion_core

ROS 非依存に近い trajectory schema, scoring concept, shared utilities。

**Status: 未実装（スケルトン）。**

生成モデル・Nav2・安全層が共有する、ROS にできるだけ依存しないコアロジックを置く。ユニットテストしやすく、runtime / training の両方から参照できることを狙う。

## 想定する責務

- Trajectory candidate のコア表現と変換（SE(2) pose 列 ⇔ control sequence）
- Kinematic Projector（diff / omni / ackermann への投影）— [../docs/architecture.md](../docs/architecture.md) §3.3
- Trajectory Scorer の概念（progress / smoothness / clearance / path alignment）— architecture §3.3
- 共有 utility（座標変換ヘルパ、時刻同期、正規化）

## Dependency Policy

ROS 2, Nav2, lightweight C++ dependencies のみ（[../docs/architecture.md](../docs/architecture.md) §12.2）。重い Python 学習依存を持ち込まない。
