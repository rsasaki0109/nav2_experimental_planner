# Deployment Strategy

> 関連: [architecture.md](architecture.md) §7 Inference、[safety.md](safety.md)、[simulation.md](simulation.md)

## 11.1 Target Platforms

| Platform | Role | 推奨 Backend |
|---|---|---|
| x86 CPU | fallback, CI, tiny model | CPU / ONNX |
| x86 NVIDIA GPU | development, benchmark, training-adjacent inference | PyTorch / ONNX / TensorRT |
| Jetson Orin | field deployment | TensorRT FP16 / INT8 |
| Edge GPU Box | warehouse AMR, service robot | TensorRT / Triton |
| Cloud GPU | training, offline benchmark | PyTorch |

NVIDIA Isaac ROS は ROS 2 上に構築された CUDA accelerated packages と AI models の集合として提供され、Jetson や workstation での robotics AI 開発・deployment を支援する。

## 11.2 Deployment Modes

| Mode | 内容 |
|---|---|
| Dev Mode | PyTorch, rich logging, slower but flexible |
| Eval Mode | ONNX, deterministic configs, benchmark |
| Production Mode | TensorRT, fixed shapes, reduced logging |
| Shadow Mode | output not executed, telemetry only |
| Safe Mode | model disabled, fallback controller only |

## 11.3 Jetson Deployment Rules

Jetson 対応では、以下を必須設計にする。

- TensorRT engine cache を device / TensorRT version / precision / model hash ごとに管理
- FP16 を標準、INT8 は calibration 済み model のみ
- GPU memory budget を manifest に明記
- CPU fallback を残す
- safety layer は GPU 非依存にする（[safety.md](safety.md) 参照）
- thermal throttling を diagnostics に出す
- model load 失敗時に robot が起動不能にならない
- camera model は optional package に分離

## 11.4 Packaging Strategy

| Artifact | 配布方針 |
|---|---|
| ROS runtime packages | apt release を目指す |
| training tools | pip / container / optional |
| model artifacts | separate release asset / model registry |
| benchmark scenarios | repository 内または dataset release |
| Docker | dev, benchmark, deployment を分ける |
| docs | GitHub Pages / docs site |

## 11.5 Staged Rollout

実機導入は必ず段階化する。

1. Offline rosbag replay
2. Closed-loop simulation
3. Shadow mode on robot
4. Low-speed supervised run
5. Limited ODD deployment
6. Wider deployment
7. Fleet telemetry feedback

**ODD** は Operational Design Domain の意味で、使用可能な環境条件を明記する。例: indoor, flat floor, max speed, sensor type, human density, lighting condition。
