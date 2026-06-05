# nav2_diffusion_safety

safety gate, collision validation integration。

**Status: 最小実装あり（ビルド & テスト通過）。最重要・GPU 非依存。**

決定論的な安全層。生成モデルの候補軌道を検証し、安全でないものを棄却する。安全判定は **GPU 非依存** で動くこと（[../docs/deployment.md](../docs/deployment.md) §11.3）。

## 現状の実装

- `safety_filter.hpp`: `SafetyFilter` 抽象インターフェースと `SafetyResult`（safe / rejection_reason）
- `kinematic_limits_filter.hpp/.cpp`: 線速度・角速度の上限超過を棄却する `KinematicLimitsFilter`（Kinematic Safety Layer、[../docs/safety.md](../docs/safety.md) §8.2）
- `footprint_collision_filter.hpp/.cpp`: `nav2_costmap_2d` の `FootprintCollisionChecker` を用い、向き付き footprint が costmap の lethal セルと衝突する候補を棄却する `FootprintCollisionFilter`（Footprint Collision Layer、§8.2）。costmap を runtime truth source として扱う（[../docs/architecture.md](../docs/architecture.md) §3.4）。呼び出し側が costmap mutex を保持すること。
- gtest（`test/test_kinematic_limits_filter.cpp`, `test/test_footprint_collision_filter.cpp`）

> **The learned planner is never the final authority.**（[../docs/safety.md](../docs/safety.md) §8.1）

## 想定する責務

- Safety Layers（[../docs/safety.md](../docs/safety.md) §8.2）: input validity / observation sanity / candidate validity / kinematic / footprint collision / dynamic risk / social / command safety
- Safety State Machine（safety §8.3、`nav2_diffusion_msgs/SafetyState`）の発行
- Nav2 Collision Monitor 連携（学習 planner の外側の非常停止層）
- Fallback Manager（safety §8.4: MPPI / RPP / rotate / stop / recovery）

## 原則

- runtime truth source は Costmap / TF / Odometry（[../docs/architecture.md](../docs/architecture.md) §3.4）。
- stale data（古い TF / costmap / odom）からの軌道は棄却（architecture §7.4）。
