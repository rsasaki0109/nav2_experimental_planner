# nav2_nd_controller

**Nearness Diagram (ND) local controller**（`nav2_core::Controller`）。

**Status: 実装あり（ビルド & lint 通過 / pluginlib 登録・closed-loop 統合テスト済み）。classical（非AI）controller。**

ND navigation（Minguez & Montano, 2004）は、ロボット周囲の障害物の**近さ（nearness）をセクタごと**に評価し、ロボットが通れる**navigable な領域（gap）**を抽出して目標方向に近い gap を選び、さらに **safety deflection**（近い障害物のある側から離れる偏向）を加えて操舵する反応的回避法。安全偏向により**回廊の中央寄せ**や**狭所のすり抜け**が自然に出るのが、ヒストグラム valley コストで選ぶ VFH+ との明確な違い。

本実装は簡易版 ND（セクタごとの最近傍距離 + 領域/gap 選択 + 対称な safety deflection）。VFH+ とは別系統の反応パラダイムで、Nav2 公式にはどちらも無い（`Nav2PlannerBattle` = Nav2 に無い planner 群の一部）。

## アルゴリズム

`nav2_nd_controller::NDController::computeVelocityCommands()`:

1. plan が空 → 停止。goal まで `goal_dist_tolerance` 以内 → 停止。
2. **目標方位**: lookahead 点を base frame に変換した方位 `target`。
3. **nearness diagram**: `active_window` 内の障害物セルをセクタへ投影し、各セクタの最近傍距離を記録。併せて、左右それぞれの近接障害物から「反対側へ押す」偏向需要（`push_left` / `push_right`）と前方最近傍距離を集計。
4. **領域選択**: 最近傍距離が `max(robot_radius+safety, security_distance)` を超えるセクタを navigable とし、`min_region_sectors` 幅以上の領域を gap とみなす。`target` の gap が navigable ならそのまま、塞がれていれば `target` に最も近い gap の方向へ（正面障害物では側方 gap を選ぶ）。
5. **safety deflection**: `deflection = gain·(push_left − push_right)` を方向に加える（左右対称なら相殺＝中央寄せ、片側のみ近ければ離れる方へ偏向）。
6. **速度**: `angular = clamp(gain·θ)`、`linear = max·scale·turn_factor·front_factor`（旋回・前方近接・goal 近接で減速、後退しない）。

> ロボット半径と security_distance でブロック判定するため、**inflation 無しの生 costmap** 前提。inflation 済みなら閾値を調整。完全に決定論的。

## closed-loop 統合テスト

`test/test_nd_controller.cpp`（稼働中の `nav2_costmap_2d::Costmap2DROS`、GPU/シム不要、静的 TF で map→base_link）:

- クリア路 → 前進（linear>0, angular≈0）
- 正面障害物（security 距離内）→ 側方 gap へ操舵しつつ前進
- **右側のみ近接障害物** → 左へ偏向（angular>0、ND の hallmark）、前方クリアで前進継続
- plan 無し → 停止
- goal 直上 → 停止

## パラメータ

| 名前 | 既定 | 意味 |
|---|---|---|
| `num_sectors` | 72 | 極座標分解能 |
| `active_window` | 2.0 | スキャン半径 [m] |
| `obstacle_threshold` | 253 | このコスト以上を障害物とみなす |
| `allow_unknown` | true | unknown を通行可能とみなすか |
| `robot_radius` | 0.2 | 衝突クリアランス半径 [m] |
| `safety_distance` | 0.1 | 追加クリアランス [m] |
| `security_distance` | 0.4 | この距離以内で偏向＆セクタをブロック |
| `min_region_sectors` | 3 | gap とみなす最小幅 |
| `lookahead_distance` | 0.6 | carrot 距離 [m] |
| `max_linear_speed` / `max_angular_speed` | 0.5 / 1.5 | 速度上限 |
| `angular_gain` | 1.5 | 比例操舵ゲイン |
| `deflection_gain` | 1.0 | safety deflection の強さ |
| `slow_distance` | 0.6 | 前方クリアランスで減速し始める距離 [m] |
| `goal_dist_tolerance` | 0.25 | この距離以内で停止 [m] |

## 使い方（例）

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_nd_controller::NDController"
      security_distance: 0.4
      deflection_gain: 1.0
```

最小例: [../nav2_diffusion_bringup/params/nd_controller_example.yaml](../nav2_diffusion_bringup/params/nd_controller_example.yaml)。

## 関連

- もう一つの反応的 local controller: [../nav2_vfh_controller/README.md](../nav2_vfh_controller/README.md)（VFH+）
- 生成型 local controller（Mode A）: [../nav2_diffusion_controller/README.md](../nav2_diffusion_controller/README.md)
- アーキテクチャ: [../docs/architecture.md](../docs/architecture.md)
