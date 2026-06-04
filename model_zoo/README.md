# model_zoo

curated model metadata, not necessarily large binaries。

検証済みモデルの metadata（model card / manifest）を集約する。原則として大きなバイナリは置かず、release asset / model registry を指す（[../docs/deployment.md](../docs/deployment.md) §11.4）。例外として、**小型かつ再現スクリプト同梱**のモデルは直接コミットしてよい（誰でもベンチを再現できるように）。

## 収録モデル

| モデル | 種別 | seam | アーティファクト | card / manifest |
|---|---|---|---|---|
| `diffusion_global_costmap_flow_v0` | flow matching（学習済み） | Mode B `PathModel`（global path） | [diffusion_global/costmap_flow.onnx](diffusion_global/costmap_flow.onnx)（≈349 KB, 直接コミット・[export.py](diffusion_global/export.py) で再現可） | [model_card.md](diffusion_global/model_card.md) / [manifest.yaml](diffusion_global/manifest.yaml) |
| `diffusion_local_costmap_flow_v0` | flow matching（学習済み） | Mode A `TrajectoryModel`（local controller） | [diffusion_local/costmap_flow.onnx](diffusion_local/costmap_flow.onnx)（≈268 KB, 直接コミット・[export.py](diffusion_local/export.py) で再現可） | [model_card.md](diffusion_local/model_card.md) / [manifest.yaml](diffusion_local/manifest.yaml) |
| `diffusion_local_costmap_transformer_v0` | transformer set-prediction（学習済み・GPU 学習） | Mode A `TrajectoryModel`（local controller） | [diffusion_local_transformer/costmap_transformer.onnx](diffusion_local_transformer/costmap_transformer.onnx)（≈224 KB, 直接コミット・[export.py](diffusion_local_transformer/export.py) で再現可） | [model_card.md](diffusion_local_transformer/model_card.md) / [manifest.yaml](diffusion_local_transformer/manifest.yaml) |

これらが「**実際に C++ 推論経路でループに入っている学習済みモデル**」（unit-test fixture ではない）。いずれも costmap を読んで全提案を空き側へ寄せる。**Mode B** は `nav2_diffusion_global_planner` で `OnnxPathModel` 経由、**Mode A** は `nav2_diffusion_controller` で `OnnxTrajectoryModel` 経由でロードされ、決定論的安全層が検証・選択する。横断比較は [../docs/planner_comparison.md](../docs/planner_comparison.md)（Mode B）/ [../docs/controller_comparison.md](../docs/controller_comparison.md)（Mode A）、限界・失敗ケースは各 model card を参照。

## ルール

- 収録モデルは **benchmark 通過済み** 必須（[../docs/architecture.md](../docs/architecture.md) §3.4 / §5.5）。研究用 placeholder は behavioural check（end-to-end テスト）でよいが、その旨を card に明記する。
- **curated と experimental を分離** し、品質低下を防ぐ（[../docs/risks.md](../docs/risks.md) §14.2）。
- 各モデルは limitations と failure cases を公開する（[../docs/risks.md](../docs/risks.md) §14.3）。
- 直接コミットするのは小型 (≲ 1 MB) かつ再現スクリプト付きのモデルに限る。大型は release asset を指す。
