# Model Zoo

> 関連: [architecture.md](architecture.md) §5.4 Manifest / §5.5 Extension Rule、[contributing.md](contributing.md)

Model Zoo は、検証済みモデルの **model card と manifest** を集約する場所です。大きなバイナリは置かず、release asset / model registry を指します（[deployment.md](deployment.md) §11.4）。

> 出荷する6つの生成系統（flow / diffusion / consistency × context-only / costmap+goal）を同一の合成回避シナリオで比較したオフライン leaderboard は [model_comparison.md](model_comparison.md) を参照（`tools/benchmark_models.py` で再現可能、CPU・決定論的）。

## 収録の条件

- **benchmark 通過済み**であること（§3.4）。
- model manifest（§5.4 全フィールド）と model card（§5.5）が揃っていること。
- limitations と failure cases が公開されていること（[risks.md](risks.md) §14.3）。
- **curated と experimental を分離**して品質低下を防ぐ（[risks.md](risks.md) §14.2）。

## テンプレート

- Manifest: [../nav2_diffusion_models/manifests/example_diffusion_local.yaml](../nav2_diffusion_models/manifests/example_diffusion_local.yaml)
- Model card: [../nav2_diffusion_models/model_card_template.md](../nav2_diffusion_models/model_card_template.md)

## Curated models

| Model | Family | Robot | Runtime | Benchmark | Card |
|---|---|---|---|---|---|
| _（まだありません）_ | - | - | - | - | - |

## Experimental models

| Model | Family | Robot | Runtime | Notes | Card |
|---|---|---|---|---|---|
| _（まだありません）_ | - | - | - | - | - |

> 注: 組み込みの `FanRolloutModel` は解析的なプレースホルダで、配布物（artifact）を持たないため Model Zoo の対象外です（[../nav2_diffusion_core/README.md](../nav2_diffusion_core/README.md)）。
