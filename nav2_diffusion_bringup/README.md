# nav2_diffusion_bringup

example launch/config for Nav2。

**Status: 未実装（スケルトン）。**

既存 Nav2 ユーザーが **Controller を差し替えるだけ** で試せる launch / param 例を提供する。試用障壁を下げることが DX 上の勝ち筋（[../docs/architecture.md](../docs/architecture.md) §15.6）。

## 想定する内容

- Nav2 標準 demo（Gazebo + TurtleBot3 導線、[../docs/simulation.md](../docs/simulation.md) §10.1）への差し替え launch
- `nav2_diffusion_controller` を使う controller_server param 例
- **Default Safe Config**: model が壊れても止まる設定（[../docs/safety.md](../docs/safety.md) §8.5）
- fallback controller（MPPI / RPP）併設の構成例
- RViz 設定（候補軌道 + 棄却理由 + best trajectory の可視化）
