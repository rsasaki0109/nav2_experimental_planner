# Risks

> 関連: [safety.md](safety.md)、[architecture.md](architecture.md) §12.2 Runtime/Training Separation

## 14.1 Technical Risks

| Risk | Impact | Mitigation |
|---|---|---|
| 推論 latency が大きい | 制御周期を守れない | few-step model, TensorRT, async inference, fallback |
| 生成軌道が衝突する | 安全問題 | deterministic safety gate, collision monitor, hard rejection |
| sim-to-real gap | 実機で不安定 | costmap-first, real bags, shadow mode, domain randomization |
| camera 依存が強すぎる | AMR 導入障壁 | camera optional, LiDAR/costmap baseline |
| stochastic output が再現不能 | benchmark 困難 | seed control, trace, candidate logging |
| 社会的ナビゲーション評価が曖昧 | 誇大主張 | social metrics を明文化、失敗例公開 |
| Nav2 version 差分 | plugin 互換性問題 | supported distro matrix, no fork |
| GPU memory 不足 | Jetson で動かない | small models, quantization, model manifest |
| OOD 環境で暴走 | 実機 risk | confidence gating, OOD detector, conservative fallback |
| costmap と raw sensor が矛盾 | 誤判断 | runtime truth source を明確化、stale/consistency check |

## 14.2 OSS Operation Risks

| Risk | Impact | Mitigation |
|---|---|---|
| 研究コード化 | 実ユーザーが離れる | runtime/training 分離、docs 重視 |
| install が重い | Star は増えても使われない | Docker, minimal CPU demo, optional GPU |
| benchmark がない | 信頼されない | MPPI 比較を初期から整備 |
| model license 不明 | 企業が使えない | model card, license separation |
| contributor が model だけ投げる | 品質崩壊 | manifest + benchmark 必須 |
| issue 対応不能 | maintain 不能 | scope 明確化、templates、roadmap |
| Nav2 upstream と乖離 | 孤立 | plugin 準拠、upstream-friendly design |
| Star 目的で過大広告 | 信頼低下 | limitations と failure cases を公開 |
| model zoo 肥大化 | 品質低下 | curated / experimental を分離 |
| CI コスト増大 | 維持困難 | CPU smoke + GPU nightly + self-hosted optional |

## 14.3 Safety / Liability Risks

この OSS は安全認証済み製品ではない。実機導入者は hardware EStop、速度制限、ODD 定義、現場 risk assessment を行う必要がある。

OSS 側が提供すべきは、認証の代替ではなく、以下である。

- 安全設計の透明性
- failure mode の明記
- fallback の標準化
- benchmark による比較
- 実機導入手順
- 危険な使い方の明示
