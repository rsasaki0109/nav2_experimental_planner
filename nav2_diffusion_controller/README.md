# nav2_diffusion_controller

Nav2 Controller Plugin integration。

**Status: スケルトン実装あり（ビルド & lint 通過 / pluginlib 登録確認済み）。v0.1 の主軸。**

`nav2_core::Controller` を実装する Nav2 Controller Server プラグイン。Global Path と Local Costmap を受け取り、生成モデルで未来軌道候補を生成し、Safety Gate を通した Best Trajectory から `cmd_vel` を出す（Mode A、[../docs/architecture.md](../docs/architecture.md) §3.2）。

## 現状の実装

`nav2_diffusion_controller::DiffusionController` がパイプラインを配線済み:

1. **提案（multimodal）**: lookahead 点へ向かう **K 個の候補軌道**を生成（**生成モデルのプレースホルダ**。現状は角速度をファン状にサンプルして直進/左/右の複数モードを作り、後で学習モデルの multimodal 出力に差し替える）
2. **入力検証**: stale-data ゲート（robot pose/odom/TF の鮮度、costmap current。§7.4 Runtime Gating）
3. **検証（候補ごと）**: `KinematicLimitsFilter`（速度上限）→ `FootprintCollisionFilter`（Local Costmap への footprint 衝突判定、costmap mutex を保持して実行）
4. **選択（scoring）**: 安全な候補を `nav2_diffusion_core` の Trajectory Scorer（goal への接近 + smoothness）で評価し best を選ぶ（§4.1 step 7）
5. **抽出 / fallback**: best 候補から `cmd_vel`。**安全候補が無い場合** は、`fallback_controller_plugin` が設定されていれば Nav2 標準 Controller（MPPI/RPP 等）へ委譲（`SafetyState::FALLBACK`、§8.4）、未設定なら stop（`SafetyState::BRAKE`）
6. **可観測性**: 全候補（`TrajectoryCandidates`、safe_flags / rejection_reasons / best_index）と `SafetyState` を publish（RViz / rosbag 用）

`ros2 run nav2_util ...` ではなく Nav2 controller_server にプラグインとしてロードされる。`ros2 plugin list` で `nav2_core::Controller` として発見されることを確認済み。

### closed-loop 統合テスト

`test/test_diffusion_controller_integration.cpp` は、稼働中の `nav2_costmap_2d::Costmap2DROS` に対して実プラグインを configure/activate し、GPU/シム無しで以下を検証する:

- クリアな costmap + 前方への global path → `cmd_vel.linear.x > 0`（前進）
- 前方 ~0.4m に lethal 障害物 → footprint ゲートが発火し stop（`cmd_vel = 0`）
- global path 無し → stop
- robot pose が古い（`data_timeout` 超過）→ stale-data ゲートで stop
- オフ軸（左上）ゴール → scorer が**旋回候補**を選択（`angular.z > 0` で前進）
- 全候補がブロック + fallback(RPP) 設定時 → fallback へ委譲して前進（stop しない）

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
| `data_timeout` | 0.5 | robot pose（odom/TF）の鮮度タイムアウト [s]。超過で stop。0 で無効 |
| `check_costmap_current` | false | costmap が current でない場合に stop（opt-in の多重防御） |
| `num_candidates` | 11 | 生成する候補軌道数（角速度ファンのサンプル数） |
| `score_progress_weight` | 1.0 | scoring: goal 接近の重み |
| `score_smoothness_weight` | 0.1 | scoring: 旋回量ペナルティの重み |
| `fallback_controller_plugin` | "" | 安全候補ゼロ時に委譲する Nav2 Controller plugin。空で無効（stop）。例: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`。そのパラメータは `<name>.fallback.*` 名前空間に置く |

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
