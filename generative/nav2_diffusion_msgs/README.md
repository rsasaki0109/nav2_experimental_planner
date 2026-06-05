# nav2_diffusion_msgs

trajectory candidates, diagnostics, benchmark result messages.

**Status: 初期メッセージ契約あり（ビルド可能な ament_cmake パッケージ）。**

このパッケージは生成型ナビゲーションの I/O 契約を定義する。実装が進む前にメッセージを固めることで、controller / safety / rviz / benchmark の各パッケージが共通の表現を共有できる。

## Messages

| Message | 対応する設計 | 概要 |
|---|---|---|
| `TrajectoryCandidate.msg` | architecture §4.4 | time-indexed SE(2) 未来 pose 列 + optional velocity + model_score |
| `TrajectoryCandidates.msg` | architecture §4.1 / §4.3 | K 個の候補 + safe_flags + rejection_reasons + best_index |
| `SafetyState.msg` | safety §8.3 | NOMINAL / CAUTIOUS / DEGRADED / FALLBACK / BRAKE / EMERGENCY_STOP / RECOVERY |

## 設計上の注意

- 内部表現は **直接 velocity 列ではなく time-indexed SE(2) trajectory**（[../docs/architecture.md](../docs/architecture.md) §4.4）。`cmd_vel` は Command Extractor が後段で導出する。
- `model_score` は学習 prior の soft signal であり、**安全を意味しない**。安全判定は deterministic safety gate（[../docs/safety.md](../docs/safety.md)）が行う。

## TODO

- benchmark result message（[../docs/benchmarking.md](../docs/benchmarking.md) §9.4 metrics）
- model telemetry / diagnostics message（architecture §4.3）
- rejection reason の enum 化検討（現状は free-text string）
