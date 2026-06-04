# model_zoo

curated model metadata, not necessarily large binaries。

検証済みモデルの metadata（model card / manifest）を集約する。原則として大きなバイナリは置かず、release asset / model registry を指す（[../docs/deployment.md](../docs/deployment.md) §11.4）。例外として、**小型かつ再現スクリプト同梱**のモデルは直接コミットしてよい（誰でもベンチを再現できるように）。

## 収録モデル

| モデル | 種別 | seam | アーティファクト | card / manifest |
|---|---|---|---|---|
| `diffusion_global_costmap_flow_v0` | flow matching（学習済み） | Mode B `PathModel`（global path） | [diffusion_global/costmap_flow.onnx](diffusion_global/costmap_flow.onnx)（≈349 KB, 直接コミット・[export.py](diffusion_global/export.py) で再現可） | [model_card.md](diffusion_global/model_card.md) / [manifest.yaml](diffusion_global/manifest.yaml) |

これがリポジトリ初の「**実際に C++ 推論経路でループに入っている学習済みモデル**」（unit-test fixture ではない）。costmap を読んで全提案を空き側へ寄せる挙動を持ち、`nav2_diffusion_global_planner` の Mode B で `nav2_diffusion_onnx::OnnxPathModel` 経由でロードされる。横断比較は [../docs/planner_comparison.md](../docs/planner_comparison.md)、限界・失敗ケースは model card を参照。

## ルール

- 収録モデルは **benchmark 通過済み** 必須（[../docs/architecture.md](../docs/architecture.md) §3.4 / §5.5）。研究用 placeholder は behavioural check（end-to-end テスト）でよいが、その旨を card に明記する。
- **curated と experimental を分離** し、品質低下を防ぐ（[../docs/risks.md](../docs/risks.md) §14.2）。
- 各モデルは limitations と failure cases を公開する（[../docs/risks.md](../docs/risks.md) §14.3）。
- 直接コミットするのは小型 (≲ 1 MB) かつ再現スクリプト付きのモデルに限る。大型は release asset を指す。
