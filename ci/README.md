# ci

CI scripts and simulation regression configs。

**Status: 未実装（スケルトン）。**

CI/CD スクリプトと simulation regression 設定（[../docs/architecture.md](../docs/architecture.md) §12.4）。

## 想定する内容

- Lint / Build Matrix（supported ROS 2 distros）
- Unit / Integration（Nav2 bringup, plugin load, lifecycle）
- Simulation Tests（headless Gazebo）, Benchmark Smoke
- Model Artifact Tests（manifest, checksum, shape）
- GPU Nightly（TensorRT / ONNX Runtime）
- コスト方針: CPU smoke + GPU nightly + self-hosted optional（[../docs/risks.md](../docs/risks.md) §14.2）
