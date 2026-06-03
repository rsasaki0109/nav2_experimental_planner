# nav2_diffusion_safety

safety gate, collision validation integration。

**Status: 未実装（スケルトン）。最重要・GPU 非依存。**

決定論的な安全層。生成モデルの候補軌道を検証し、安全でないものを棄却する。安全判定は **GPU 非依存** で動くこと（[../docs/deployment.md](../docs/deployment.md) §11.3）。

> **The learned planner is never the final authority.**（[../docs/safety.md](../docs/safety.md) §8.1）

## 想定する責務

- Safety Layers（[../docs/safety.md](../docs/safety.md) §8.2）: input validity / observation sanity / candidate validity / kinematic / footprint collision / dynamic risk / social / command safety
- Safety State Machine（safety §8.3、`nav2_diffusion_msgs/SafetyState`）の発行
- Nav2 Collision Monitor 連携（学習 planner の外側の非常停止層）
- Fallback Manager（safety §8.4: MPPI / RPP / rotate / stop / recovery）

## 原則

- runtime truth source は Costmap / TF / Odometry（[../docs/architecture.md](../docs/architecture.md) §3.4）。
- stale data（古い TF / costmap / odom）からの軌道は棄却（architecture §7.4）。
