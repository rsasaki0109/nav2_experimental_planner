# model_zoo

curated model metadata, not necessarily large binaries。

検証済みモデルの metadata（model card / manifest）を集約する。原則として大きなバイナリは置かず、release asset / model registry を指す（[../docs/deployment.md](../docs/deployment.md) §11.4）。例外として、**小型かつ再現スクリプト同梱**のモデルは直接コミットしてよい（誰でもベンチを再現できるように）。

## 収録モデル

| モデル | 種別 | seam | アーティファクト | card / manifest |
|---|---|---|---|---|
| `diffusion_global_costmap_flow_v0` | flow matching（学習済み） | Mode B `PathModel`（global path） | [diffusion_global/costmap_flow.onnx](diffusion_global/costmap_flow.onnx)（≈349 KB, 直接コミット・[export.py](diffusion_global/export.py) で再現可） | [model_card.md](diffusion_global/model_card.md) / [manifest.yaml](diffusion_global/manifest.yaml) |
| `diffusion_global_costmap_transformer_v0` | transformer set-prediction（学習済み・GPU 学習） | Mode B `PathModel`（global path） | [diffusion_global_transformer/costmap_transformer.onnx](diffusion_global_transformer/costmap_transformer.onnx)（≈224 KB, 直接コミット・[export.py](diffusion_global_transformer/export.py) で再現可） | [model_card.md](diffusion_global_transformer/model_card.md) / [manifest.yaml](diffusion_global_transformer/manifest.yaml) |
| `diffusion_global_costmap_recurrent_v0` | recurrent（GRU 自己回帰ロールアウト・学習済み・GPU 学習） | Mode B `PathModel`（global path） | [diffusion_global_recurrent/costmap_recurrent.onnx](diffusion_global_recurrent/costmap_recurrent.onnx)（≈1.0 MB, 直接コミット・[export.py](diffusion_global_recurrent/export.py) で再現可） | [model_card.md](diffusion_global_recurrent/model_card.md) / [manifest.yaml](diffusion_global_recurrent/manifest.yaml) |
| `diffusion_local_costmap_flow_v0` | flow matching（学習済み） | Mode A `TrajectoryModel`（local controller） | [diffusion_local/costmap_flow.onnx](diffusion_local/costmap_flow.onnx)（≈268 KB, 直接コミット・[export.py](diffusion_local/export.py) で再現可） | [model_card.md](diffusion_local/model_card.md) / [manifest.yaml](diffusion_local/manifest.yaml) |
| `diffusion_local_costmap_transformer_v0` | transformer set-prediction（学習済み・GPU 学習） | Mode A `TrajectoryModel`（local controller） | [diffusion_local_transformer/costmap_transformer.onnx](diffusion_local_transformer/costmap_transformer.onnx)（≈224 KB, 直接コミット・[export.py](diffusion_local_transformer/export.py) で再現可） | [model_card.md](diffusion_local_transformer/model_card.md) / [manifest.yaml](diffusion_local_transformer/manifest.yaml) |
| `diffusion_local_costmap_recurrent_v0` | recurrent（GRU 自己回帰ロールアウト・学習済み・GPU 学習） | Mode A `TrajectoryModel`（local controller） | [diffusion_local_recurrent/costmap_recurrent.onnx](diffusion_local_recurrent/costmap_recurrent.onnx)（≈592 KB, 直接コミット・[export.py](diffusion_local_recurrent/export.py) で再現可） | [model_card.md](diffusion_local_recurrent/model_card.md) / [manifest.yaml](diffusion_local_recurrent/manifest.yaml) |

これらが「**実際に C++ 推論経路でループに入っている学習済みモデル**」（unit-test fixture ではない）。いずれも costmap を読んで全提案を空き側へ寄せる。**Mode B** は `nav2_diffusion_global_planner` で `OnnxPathModel` 経由、**Mode A** は `nav2_diffusion_controller` で `OnnxTrajectoryModel` 経由でロードされ、決定論的安全層が検証・選択する。横断比較は [../docs/planner_comparison.md](../docs/planner_comparison.md)（Mode B）/ [../docs/controller_comparison.md](../docs/controller_comparison.md)（Mode A）、限界・失敗ケースは各 model card を参照。

> **`diffusion_global_costmap_transformer_v0` の位置づけ（正直なスコープ）**:
> `planner_benchmark`（8 コース）上で **footprint 検証付き *off-centre gap* を純生成で貫通する初の
> Mode B モデル**（fallback なし、実 C++ で検証）。さらに **dead-ahead の隙間（*centred gap* /
> *narrow gap* / 直進 2 連の *double gate*）も同時に貫通**し、*clear* / *side obstacle* も維持。
> flow / recurrent（16 次元 CNN embedding）は off-axis gap では *no path*。効くのは2点の合わせ技:
> ① token attention で off-centre slot に提案を**向ける**（A/B + C++ テスト
> `CuratedZooTransformerAimsAtOffCentreSlot`）、② **微分可能 footprint クリアランス損失**
> （`export.py` の `footprint` / `blur_sigma`）で提案を **validator が通す形に最適化**。小容量では
> off-centre と dead-ahead がトレードオフだったが、**容量増（dim64/h8/l3）+ centred tri-mix で解消**
> （詳細は generative_limits）。**残る bound**: *far off-centre gap*（off-axis スロット前方約 3 m）と
> *slalom* は純生成では *no path*。完全性保証は引き続き **hybrid プランナ**。詳細は
> [model_card](diffusion_global_transformer/model_card.md) と
> [../docs/generative_limits.md](../docs/generative_limits.md)。

> **`diffusion_global_costmap_recurrent_v0` の位置づけ（正直なスコープ）**:
> Mode B 3 系統目（flow / transformer に GRU 自己回帰ロールアウトを追加）。flow と
> **同じ 16 次元 CNN embedding** に条件付けするため、**一方向障害物の空き側を選ぶ点で
> flow と同等の peer**（*clear* / *side obstacle* を解き、*off-centre gap* / *slalom* は
> *no path*）。transformer のような off-centre slot へのエイムは**しない**（token attention が
> 必要）。意義は seam が Mode A・Mode B 双方で同一ファミリ（逐次帰納バイアス）を運べることの
> 実証。コストは H=12 の逐次ロールアウトで **Mode B 3 系統中レイテンシ最大**（flow 4 step /
> transformer 1 forward）。C++ 方向テスト `CuratedZooPathRecurrentVeersAwayFromObstacle` 付き。

## ルール

- 収録モデルは **benchmark 通過済み** 必須（[../docs/architecture.md](../docs/architecture.md) §3.4 / §5.5）。研究用 placeholder は behavioural check（end-to-end テスト）でよいが、その旨を card に明記する。
- **curated と experimental を分離** し、品質低下を防ぐ（[../docs/risks.md](../docs/risks.md) §14.2）。
- 各モデルは limitations と failure cases を公開する（[../docs/risks.md](../docs/risks.md) §14.3）。
- 直接コミットするのは小型 (≲ 1 MB) かつ再現スクリプト付きのモデルに限る。大型は release asset を指す。
