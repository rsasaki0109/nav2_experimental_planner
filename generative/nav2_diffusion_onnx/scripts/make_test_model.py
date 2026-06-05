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
Export a tiny ONNX model used to verify the OnnxTrajectoryModel C++ backend.

The model maps a context vector [goal_x, goal_y, linear_speed, max_angular] to a
[1, K, H, 3] trajectory tensor (K candidates, H steps, x/y/yaw). It is a single
linear layer with fixed weights -- not a trained planner, just a deterministic
fixture exercising the C++ tensor plumbing and reshape.
"""

import sys

import torch
from torch import nn

NUM_CANDIDATES = 3
HORIZON = 10


class TinyTrajectoryModel(nn.Module):
    """Linear context -> [1, K, H, 3] trajectory tensor."""

    def __init__(self):
        super().__init__()
        torch.manual_seed(0)
        self.linear = nn.Linear(4, NUM_CANDIDATES * HORIZON * 3)

    def forward(self, context):
        """Map a [1, 4] context to a [1, K, H, 3] trajectory tensor."""
        flat = self.linear(context)
        return flat.reshape(1, NUM_CANDIDATES, HORIZON, 3)


def main():
    """Export the fixture model to the path given as argv[1]."""
    out_path = sys.argv[1]
    model = TinyTrajectoryModel().eval()
    dummy = torch.zeros(1, 4)
    torch.onnx.export(
        model, dummy, out_path,
        input_names=['context'], output_names=['trajectories'],
        opset_version=17)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
