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
Export a tiny 2-input ONNX model to verify the costmap-conditioned OnnxPathModel.

Maps (context [1, 2] = [goal_distance, 0], costmap [1, 1, S, S]) to a [1, K, H, 2]
tensor of aligned candidate paths. It is not trained: it measures which lateral
half of the patch holds more obstacle mass and bows every candidate toward the
opposite (free) side, so the C++ test can confirm the costmap input actually
steers the output (and the frame transform/anchoring around it).
"""

import sys

import torch
from torch import nn

PATH_K = 5
PATH_H = 12
SIZE = 24


class TinyCostmapPathModel(nn.Module):
    """Bow candidates toward the obstacle-free lateral half of the patch."""

    def __init__(self):
        super().__init__()
        t = torch.linspace(0.0, 1.0, PATH_H)
        self.register_buffer('t', t)
        self.register_buffer('bump', t * (1.0 - t) * 4.0)
        self.register_buffer('amps', torch.linspace(0.5, 1.0, PATH_K) * 0.9)

    def forward(self, context, costmap):
        """Map context [1,2] + costmap [1,1,S,S] to aligned paths [1,K,H,2]."""
        d = context[:, 0:1]                                # [1, 1] goal distance
        # col >= S/2 is the +y (left) half; compare obstacle mass left vs right.
        left = costmap[:, :, :, SIZE // 2:].sum(dim=(1, 2, 3))   # [1]
        right = costmap[:, :, :, :SIZE // 2].sum(dim=(1, 2, 3))  # [1]
        gap_sign = torch.sign(right - left + 1e-6).reshape(-1, 1, 1)  # +1 -> veer +y

        xs = self.t.reshape(1, 1, PATH_H) * d.reshape(-1, 1, 1)
        xs = xs.expand(-1, PATH_K, -1)                     # [1, K, H]
        ys = (gap_sign * self.amps.reshape(1, PATH_K, 1) * self.bump.reshape(1, 1, PATH_H))
        return torch.stack([xs, ys], dim=-1)              # [1, K, H, 2]


def main():
    """Export the fixture model to the path given as argv[1]."""
    out_path = sys.argv[1]
    model = TinyCostmapPathModel().eval()
    dummy_ctx = torch.zeros(1, 2)
    dummy_map = torch.zeros(1, 1, SIZE, SIZE)
    torch.onnx.export(
        model, (dummy_ctx, dummy_map), out_path,
        input_names=['context', 'costmap'], output_names=['paths'],
        opset_version=17)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
