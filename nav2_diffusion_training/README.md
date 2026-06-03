# nav2_diffusion_training

dataset, training, export pipeline。

**Status: 未実装（スケルトン）。runtime から分離。**

学習パイプライン（[../docs/training.md](../docs/training.md)）。Python / PyTorch / dataset tools。**runtime package には持ち込まない**（[../docs/architecture.md](../docs/architecture.md) §12.2）。

## 想定する内容

- Data Collection / Normalization / Label Generation / Curation（training §6.1）
- Dataset Schema 実装（training §6.3）
- Training objectives（training §6.4）
- Open-loop / Closed-loop eval（training §6.1）
- Deployment Export（ONNX / TensorRT / quantized — [../docs/deployment.md](../docs/deployment.md)）

依存は pip / container / optional とし、runtime とは別管理にする。
