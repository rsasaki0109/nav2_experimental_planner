# nav2_diffusion_rviz_plugins

candidate trajectory visualization。

**Status: 未実装（スケルトン）。**

RViz で候補軌道・棄却理由・best trajectory・safety state を可視化する。「すべての候補軌道は可視化・記録できる」という Non-Negotiable Rule（[../docs/architecture.md](../docs/architecture.md) §3.4）を支える。

## 想定する内容

- `nav2_diffusion_msgs/TrajectoryCandidates` の display（best はハイライト、棄却候補は理由付きで別色）
- `nav2_diffusion_msgs/SafetyState` の状態表示
- benchmark / shadow mode のデバッグ補助
