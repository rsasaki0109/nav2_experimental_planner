# nav2_experimental_global_planner (package)

Nav2 Planner Plugin integration。

**Status: 未実装（スケルトン）。v1.0 以降。**

> 注: `Nav2PlannerBattle` の **Mode B (GlobalPlanner) プラグイン** スケルトン。v0.1 の主軸は Controller 側（`nav2_diffusion_controller`）であることに注意。

`nav2_core::GlobalPlanner` を実装する Nav2 Planner Server プラグイン。global path または中距離 subgoal sequence を生成する（Mode B、[../docs/architecture.md](../docs/architecture.md) §3.2）。

## 用途

狭路、U字罠、長い迂回、semantic prior が効く大域的判断。

## 注意

Planner Server だけでは `cmd_vel` を出さない。必ず Controller Plugin Mode（`nav2_diffusion_controller`）と組み合わせる。
