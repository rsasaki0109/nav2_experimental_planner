# nav2_diffusion_controller

Nav2 Controller Plugin integration。

**Status: 未実装（スケルトン）。v0.1 の主軸。**

`nav2_core::Controller` を実装する Nav2 Controller Server プラグイン。Global Path と Local Costmap を受け取り、生成モデルで未来軌道候補を生成し、Safety Gate を通した Best Trajectory から `cmd_vel` を出す（Mode A、[../docs/architecture.md](../docs/architecture.md) §3.2）。

## v0.1 スコープ

- 入力: Goal, Global Path snippet, Local Costmap, Odometry
- 出力: trajectory candidates（`nav2_diffusion_msgs`）, best trajectory, `cmd_vel`
- 軽量 diffusion trajectory generator（camera 不要）
- hard collision gate + velocity limits + stop fallback（[../docs/safety.md](../docs/safety.md)）

## 統合方針

- **Nav2 を fork しない。** pluginlib export で Controller Server に差し込む。
- Neural model は直接 `cmd_vel` を publish しない。必ず Safety Gate（`nav2_diffusion_safety`）と Command Extractor を経由する。
- 推論は async-first・deadline-aware（[../docs/architecture.md](../docs/architecture.md) §7）。間に合わなければ fallback。

## 関連

- 推論バックエンド抽象化: architecture §7.2（PyTorch / ONNX Runtime / TensorRT）
- fallback: [../docs/safety.md](../docs/safety.md) §8.4（MPPI / RPP / stop）
