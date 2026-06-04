# nav2_vfh_controller

**Vector Field Histogram Plus (VFH+) local controller**（`nav2_core::Controller`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）controller。**

VFH+（Ulrich & Borenstein, 1998）は、ロボット周囲の障害物を**極座標ヒストグラム**に積み、blocked / free の角度セクタに二値化して、**free な valley（隙間）**のうち目標方位・旋回量・操舵の滑らかさを最もよく満たす方向へ操舵する**反応的（reactive）**な局所回避法。各障害物をロボット半径ぶん**拡大**してから二値化するので、free なセクタは実際に通過可能になる。軌道ロールアウトを持たない分、雑然とした空間で安価かつ堅牢。

Nav2 公式の局所コントローラ（DWB / MPPI / Regulated Pure Pursuit）は最適化ベースで、**VFH 系は無い**（`nav2_experimental_planner` = Nav2 に無い planner 群の一部）。これは本リポジトリ初の **Controller（Mode A）側**の classical 実装で、生成型 [nav2_diffusion_controller](../nav2_diffusion_controller/README.md) と同じ Controller seam に並ぶ。

## アルゴリズム

`nav2_vfh_controller::VFHController::computeVelocityCommands()`:

1. plan が空 → 停止。goal まで `goal_dist_tolerance` 以内 → 停止（goal checker が action を終了）。
2. **目標方位**: global plan 上の lookahead 点を base frame に変換し、その方位 `target` を得る。
3. **極座標ヒストグラム**: `active_window` 内の障害物セル（cost ≥ `obstacle_threshold`）を、距離に応じた拡大角 γ=asin((robot_radius+safety)/d) ぶん広げて、覆うセクタを blocked にする。
4. **方向選択**: `target` セクタが free ならそのまま `target` 方位へ。塞がっていれば、free セクタの中でコスト `μ_target·Δ(θ,target) + μ_heading·|θ| + μ_smooth·Δ(θ,prev)` 最小の方向を選ぶ。free が無ければその場旋回で開口を探す。
5. **速度**: `angular = clamp(gain·θ)`、`linear = max_linear · speed_scale · max(0, 1−|θ|/90°)`（旋回・goal 近接で減速、後退しない）。

> 拡大角でロボット半径を考慮するため、**inflation 無しの生 costmap** 前提なら `robot_radius` を実値に。inflation 済み costmap で inscribed を障害物に使う場合は二重計上を避けるため `robot_radius` を小さく（0 付近）に設定する。完全に決定論的。

## closed-loop 統合テスト

`test/test_vfh_controller.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要、静的 TF で map→base_link を供給）:

- クリア路 → 前進（linear>0, angular≈0）
- **正面に障害物・側方は free** → free valley へ操舵（angular≠0）しつつ前進（後退しない）
- plan 無し → 停止
- goal 直上 → 停止

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `num_sectors` | 72 | ヒストグラム分解能（360°のセクタ数） |
| `active_window` | 2.0 | ヒストグラム半径 [m] |
| `obstacle_threshold` | 253 | このコスト以上を障害物とみなす |
| `allow_unknown` | true | unknown を通行可能とみなすか |
| `robot_radius` | 0.2 | 障害物拡大半径 [m] |
| `safety_distance` | 0.1 | 追加の拡大マージン [m] |
| `min_valley_sectors` | 1 | 候補セクタ周囲に要求する free 幅 |
| `lookahead_distance` | 0.6 | plan 上の carrot 距離 [m] |
| `max_linear_speed` | 0.5 | [m/s] |
| `max_angular_speed` | 1.5 | [rad/s] |
| `angular_gain` | 1.5 | 比例操舵ゲイン |
| `goal_dist_tolerance` | 0.25 | この距離以内で停止 [m] |
| `mu_target` / `mu_heading` / `mu_smooth` | 5.0 / 2.0 / 2.0 | 方向コストの重み |

## 使い方（例）

controller_server の `FollowPath` plugin を差し替える:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_vfh_controller::VFHController"
      active_window: 2.0
      robot_radius: 0.2
```

最小例: [../nav2_diffusion_bringup/params/vfh_controller_example.yaml](../nav2_diffusion_bringup/params/vfh_controller_example.yaml)。

## 関連

- 生成型 local controller（Mode A）: [../nav2_diffusion_controller/README.md](../nav2_diffusion_controller/README.md)
- classical な GlobalPlanner 群: [../nav2_rrt_planner](../nav2_rrt_planner), [../nav2_prm_planner](../nav2_prm_planner), [../nav2_dstar_lite_planner](../nav2_dstar_lite_planner), [../nav2_jps_planner](../nav2_jps_planner), [../nav2_lazy_theta_star_planner](../nav2_lazy_theta_star_planner), [../nav2_ara_star_planner](../nav2_ara_star_planner), [../nav2_visibility_graph_planner](../nav2_visibility_graph_planner)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
