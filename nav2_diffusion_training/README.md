# nav2_diffusion_training

dataset, training, export pipeline。

**Status: dataset builder あり（ament_python / pytest 通過）。runtime から分離。**

学習パイプライン（[../docs/training.md](../docs/training.md)）。Python / dataset tools。**runtime package には持ち込まない**（[../docs/architecture.md](../docs/architecture.md) §12.2）。

## 現状の実装

- `nav2_diffusion_training.dataset`: 記録した SE(2) トラック（`/odom` 等）から学習サンプルを生成
  - `TrackState`（time, x, y, yaw）
  - `build_samples(track, history, horizon, stride)`: 各 anchor で **observation_window（過去の絶対 pose 列）** と **action_label（base frame に変換した未来軌道）** を生成（§6.3 schema / §4.4 SE(2)）
  - `save_jsonl(samples, path)`: JSON Lines 書き出し
  - stdlib のみ（依存軽量）
- `nav2_diffusion_training.rosbag_io`: `track_from_bag(bag_uri, topic='/odom')` — rosbag2 の odometry トピックを `TrackState` 列へ取り込む薄い adapter（§6.2）。`rosbag2_py` / `rclpy` serialization を使用。dataset 本体とは別モジュールなので、rosbag2_py 無しでも dataset は使える。
- `nav2_diffusion_training.experts`: sim 不要の rule-based expert（§6.5）。`unicycle_to_goal(start, goal_x, goal_y, ...)` が start→goal を unicycle 追従する expert track を生成。`build_samples` に渡せば**合成 imitation データセット**を sim 無しで作れる。
- pytest（`test/test_dataset.py`, `test/test_rosbag_io.py`, `test/test_experts.py`）+ ament lint（copyright / flake8 / pep257）

```python
from nav2_diffusion_training.rosbag_io import track_from_bag
from nav2_diffusion_training import build_samples, save_jsonl

track = track_from_bag('my_run_bag', topic='/odom')
save_jsonl(build_samples(track, history=4, horizon=20), 'dataset.jsonl')
```

## 想定する内容

- Data Collection / Normalization / Label Generation / Curation（training §6.1）
- Dataset Schema 実装（training §6.3）
- Training objectives（training §6.4）
- Open-loop / Closed-loop eval（training §6.1）
- Deployment Export（ONNX / TensorRT / quantized — [../docs/deployment.md](../docs/deployment.md)）

依存は pip / container / optional とし、runtime とは別管理にする。
