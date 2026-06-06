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

### オフラインで開く / 動画化（live ROS なしでも可）

live な ROS / bridge が無くても、収録済みの **MCAP** を Foxglove Studio で直接開ける。
[tools/foxglove_mcap_demo.py](../tools/foxglove_mcap_demo.py) は**実 Mode B パイプライン**（出荷
`PathFlowPlanner` が start→goal の候補を提案 → costmap 検証 → 最短安全パス選択）を障害物
スイープしながら ROS 2 メッセージとして [docs/mode_b_demo.mcap](mode_b_demo.mcap) に書き出す
（`OccupancyGrid` / `Path` / `PoseArray` / `PoseStamped`、24 フレーム・約 2.3 s）。
identity の `/tf_static`（`map`→`base_link`）も収録するので、ビューアの 3D パネルが座標
フレームを得て即描画できる（無いと「no frames」で何も出ない）。

```bash
pip install torch mcap mcap-ros2-support
PYTHONPATH=generative/nav2_diffusion_training python3 tools/foxglove_mcap_demo.py
```

開き方は 2 通り（どちらも同じ MCAP）:

- **Web 版（インストール不要・一番手軽）**: ブラウザで <https://app.foxglove.dev> を開き、
  **Open local file** から `docs/mode_b_demo.mcap` を選ぶだけ。アプリ導入なしで閲覧できる。
- **デスクトップ版**: Foxglove Studio で **File → Open local file**。

どちらでも **3D** パネルを追加して `/local_costmap`・`/path_best`（選択パス）・`/candidates_safe` /
`/candidates_rejected`・`/goal_pose` を表示。タイムラインを再生すると、障害物の移動に追従して
選択パス（緑）が左右へ切り替わる。**Export → Video** でそのまま動画化できる。

> 正直なスコープ: これは**モデル出力を可搬ファイルに収録したもの**で、Gazebo の閉ループ実走
> ではない（このリポジトリのサンドボックスは inter-process DDS 不通かつ画面なしのため、
> Foxglove の画面録画自体はここでは生成できない）。閉ループ実走は実 ROS ホストで上の
> `foxglove.launch.py` を使う（[simulation.md](simulation.md) §10.5）。

## RViz

RViz では標準の **MarkerArray** ディスプレイで `/candidate_markers` を、**Marker** で `/safety_state_marker` を追加すれば同じ可視化が得られる（[../nav2_diffusion_rviz_plugins/README.md](../generative/nav2_diffusion_rviz_plugins/README.md)）。`candidate_markers` ノードは上記 Foxglove launch でも起動するので、RViz と併用も可能。
