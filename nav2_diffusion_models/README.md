# nav2_diffusion_models

model manifest examples and lightweight test models。

**Status: 未実装（スケルトン）。**

CI とサンプル用の軽量モデルと、model manifest の例を置く。大きなバイナリは置かない（[../.gitignore](../.gitignore) でモデル拡張子を除外し、配布は release asset / model registry 経由 — [../docs/deployment.md](../docs/deployment.md) §11.4）。

## 想定する内容

- Model Manifest の例（[../docs/architecture.md](../docs/architecture.md) §5.4 の全 field）
- shape / checksum 検証用の tiny test model
- model card テンプレート（[../docs/architecture.md](../docs/architecture.md) §5.5 Extension Rule）

## ルール

Model Zoo / 同梱モデルは **benchmark 通過済み** でなければならない（architecture §3.4）。
