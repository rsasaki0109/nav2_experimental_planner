# Copyright 2026 nav2_experimental_planner contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Train -> ONNX export -> onnxruntime load, verifying the I/O contract.

Skipped automatically when PyTorch / onnxruntime are not installed (e.g. CI).
"""

import os

import pytest

torch = pytest.importorskip('torch')


def test_train_and_export_matches_backend_contract(tmp_path):
    """Training exports an ONNX model with the [1, K, H, 3] output contract."""
    from nav2_diffusion_training.train import train_and_export

    path = os.path.join(str(tmp_path), 'planner.onnx')
    train_and_export(path, num_samples=8, epochs=2)
    assert os.path.exists(path)

    ort = pytest.importorskip('onnxruntime')
    import numpy as np
    session = ort.InferenceSession(path, providers=['CPUExecutionProvider'])
    output = session.run(None, {'context': np.zeros((1, 4), dtype=np.float32)})[0]

    # Matches nav2_diffusion_onnx::OnnxTrajectoryModel: [1, K, H, 3].
    assert output.shape == (1, 3, 10, 3)


def test_training_reduces_loss(tmp_path):
    """A few epochs reduce the MSE loss on the synthetic dataset."""
    from nav2_diffusion_training.train import (
        make_synthetic_dataset, TinyPlanner)
    from torch import nn

    torch.manual_seed(0)
    model = TinyPlanner()
    inputs, targets = make_synthetic_dataset(8)
    loss_fn = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.05)

    first = loss_fn(model(inputs), targets).item()
    for _ in range(20):
        optimizer.zero_grad()
        loss = loss_fn(model(inputs), targets)
        loss.backward()
        optimizer.step()
    assert loss.item() < first
