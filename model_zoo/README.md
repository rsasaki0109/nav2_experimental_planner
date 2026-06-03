# model_zoo

curated model metadata, not necessarily large binaries。

**Status: 未実装（スケルトン）。**

検証済みモデルの metadata（model card / manifest）を集約する。大きなバイナリは置かず、release asset / model registry を指す（[../docs/deployment.md](../docs/deployment.md) §11.4）。

## ルール

- 収録モデルは **benchmark 通過済み** 必須（[../docs/architecture.md](../docs/architecture.md) §3.4 / §5.5）。
- **curated と experimental を分離** し、品質低下を防ぐ（[../docs/risks.md](../docs/risks.md) §14.2）。
- 各モデルは limitations と failure cases を公開する（[../docs/risks.md](../docs/risks.md) §14.3）。
