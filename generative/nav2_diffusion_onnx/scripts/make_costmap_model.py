# Copyright 2026 Nav2PlannerBattle contributors
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
Export a tiny TWO-INPUT ONNX fixture to verify the costmap-conditioned path.

The model takes context [1, 4] and an egocentric costmap [1, 1, S, S] and returns
a [1, K, H, 3] trajectory tensor. Random weights -- it only exercises the C++
backend's two-input plumbing, not learned behavior.
"""

import sys

import onnx
import torch
from torch import nn

SIZE = 16
NUM_CANDIDATES = 3
HORIZON = 10


class TinyCostmapModel(nn.Module):
    """context [1,4] + costmap [1,1,S,S] -> [1, K, H, 3]."""

    def __init__(self):
        super().__init__()
        torch.manual_seed(0)
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 4, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d(1), nn.Flatten())
        self.linear = nn.Linear(4 + 4, NUM_CANDIDATES * HORIZON * 3)

    def forward(self, context, costmap):
        """Encode the costmap, concat with context, and reshape to [1,K,H,3]."""
        embed = self.cnn(costmap)
        flat = self.linear(torch.cat([context, embed], dim=-1))
        return flat.reshape(1, NUM_CANDIDATES, HORIZON, 3)


def main():
    """Export the fixture to argv[1]."""
    out_path = sys.argv[1]
    model = TinyCostmapModel().eval()
    torch.onnx.export(
        model, (torch.zeros(1, 4), torch.zeros(1, 1, SIZE, SIZE)), out_path,
        input_names=['context', 'costmap'], output_names=['trajectories'],
        opset_version=18)
    onnx.save_model(onnx.load(out_path), out_path, save_as_external_data=False)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
