# Visualization

> 関連: [architecture.md](architecture.md)、[getting_started.md](getting_started.md)、[../nav2_diffusion_rviz_plugins](../generative/nav2_diffusion_rviz_plugins)

生成モデルの提案・安全層の判定・実行コマンドを可視化するための RViz / Foxglove セットアップ。

## 何が publish されるか

| トピック | 型 | 出どころ |
|---|---|---|
| `<plugin>/trajectory_candidates` | `nav2_diffusion_msgs/TrajectoryCandidates` | DiffusionController（全候補 + safe_flags + rejection_reasons + best_index） |
| `<plugin>/safety_state` | `nav2_diffusion_msgs/SafetyState` | DiffusionController（OK / FALLBACK / BRAKE など + 理由） |
| `/candidate_markers` | `visualization_msgs/MarkerArray` | `candidate_markers` ノード（best=緑 / safe=青 / rejected=赤 + 棄却理由テキスト） |
| `/safety_state_marker` | `visualization_msgs/Marker` | `candidate_markers` ノード（安全状態のラベル） |
| `/cmd_vel`, `/plan`, `/local_costmap/costmap` 等 | 標準 Nav2 | controller_server / planner_server |

`<plugin>` は通常 `FollowPath`（controller_server 名前空間下）。

`candidate_markers` ノードはカスタムメッセージを標準の `MarkerArray` / `Marker` に変換するため、RViz でも Foxglove でも追加プラグイン無しで 3D 表示できる。

## Foxglove Studio

`foxglove_bridge`（`ros-${ROS_DISTRO}-foxglove-bridge`）と marker 変換ノードをまとめて起動:

```bash
ros2 launch nav2_diffusion_bringup foxglove.launch.py
# 別ホストで bridge を動かす場合:
ros2 launch nav2_diffusion_bringup foxglove.launch.py use_bridge:=false
```

引数:

| 引数 | 既定 | 意味 |
|---|---|---|
| `use_bridge` | `true` | `foxglove_bridge` を起動するか |
| `port` | `8765` | bridge の WebSocket ポート |
| `candidates_topic` | `/controller_server/FollowPath/trajectory_candidates` | 変換元の候補トピック |
| `safety_topic` | `/controller_server/FollowPath/safety_state` | 変換元の安全状態トピック |

Foxglove Studio で `ws://<host>:8765` に接続し、**Layout → Import from file** で
[../nav2_diffusion_bringup/foxglove/nav2_diffusion_layout.json](../generative/nav2_diffusion_bringup/foxglove/nav2_diffusion_layout.json)
を読み込むと、以下のパネルが揃う:

- **3D**: costmap（local/global）、`/plan`、`/candidate_markers`（候補軌道）、`/safety_state_marker`、`/scan`
- **Raw Messages**: `/safety_state`（状態と理由の生値）
- **State Transitions**: `/safety_state.state`（OK ↔ FALLBACK ↔ BRAKE の遷移を時系列で）
- **Plot**: `/cmd_vel.linear.x` と `/cmd_vel.angular.z`

> トピック名がデフォルトと異なる場合は launch 引数で合わせるか、Foxglove 各パネルの設定で購読トピックを変更する。

## RViz

RViz では標準の **MarkerArray** ディスプレイで `/candidate_markers` を、**Marker** で `/safety_state_marker` を追加すれば同じ可視化が得られる（[../nav2_diffusion_rviz_plugins/README.md](../generative/nav2_diffusion_rviz_plugins/README.md)）。`candidate_markers` ノードは上記 Foxglove launch でも起動するので、RViz と併用も可能。
