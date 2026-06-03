# nav2_diffusion_controller

Nav2 Controller Plugin integration。

**Status: スケルトン実装あり（ビルド & lint 通過 / pluginlib 登録確認済み）。v0.1 の主軸。**

`nav2_core::Controller` を実装する Nav2 Controller Server プラグイン。Global Path と Local Costmap を受け取り、生成モデルで未来軌道候補を生成し、Safety Gate を通した Best Trajectory から `cmd_vel` を出す（Mode A、[../docs/architecture.md](../docs/architecture.md) §3.2）。

## 現状の実装

`nav2_diffusion_controller::DiffusionController` がパイプラインを配線済み:

1. **提案**: lookahead 点へ向かう候補軌道を生成（**生成モデルのプレースホルダ**。現状は pure-pursuit 風の単一候補で、後で学習モデルに差し替える）
2. **検証**: `KinematicLimitsFilter`（速度上限）→ `FootprintCollisionFilter`（Local Costmap への footprint 衝突判定、costmap mutex を保持して実行）の 2 段ゲート
3. **抽出**: 安全なら `cmd_vel`、**安全候補が無ければ stop（fallback）**
4. **可観測性**: 候補軌道（`TrajectoryCandidates`）と `SafetyState` を publish（RViz / rosbag 用）

`ros2 run nav2_util ...` ではなく Nav2 controller_server にプラグインとしてロードされる。`ros2 plugin list` で `nav2_core::Controller` として発見されることを確認済み。

### 使い方（例）

[../nav2_diffusion_bringup/params/diffusion_controller_example.yaml](../nav2_diffusion_bringup/params/diffusion_controller_example.yaml) を参照。controller_server の `FollowPath` plugin を `nav2_diffusion_controller::DiffusionController` に差し替えるだけ。

### パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `lookahead_distance` | 0.6 | carrot 点までの距離 [m] |
| `desired_linear_speed` | 0.3 | 目標前進速度 [m/s] |
| `max_linear_speed` | 0.5 | 線速度上限 [m/s]（safety gate にも使用） |
| `max_angular_speed` | 1.0 | 角速度上限 [rad/s]（safety gate にも使用） |
| `horizon` | 2.0 | 候補軌道の予測時間 [s] |
| `time_step` | 0.1 | 候補軌道の離散化刻み [s] |
| `transform_tolerance` | 0.1 | TF 変換許容時間 [s] |
| `consider_unknown_lethal` | false | costmap の unknown セルを衝突扱いするか |

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
