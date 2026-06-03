# docker

dev, benchmark, deployment containers。

**Status: 未実装（スケルトン）。**

install 失敗を減らすための Docker 環境（[../docs/architecture.md](../docs/architecture.md) §15.6、[../docs/deployment.md](../docs/deployment.md) §11.4）。用途ごとに分ける。

- **dev**: PyTorch + rich logging（[../docs/deployment.md](../docs/deployment.md) §11.2 Dev Mode）
- **benchmark**: ONNX + deterministic config
- **deployment**: TensorRT + fixed shapes（Jetson 含む）
