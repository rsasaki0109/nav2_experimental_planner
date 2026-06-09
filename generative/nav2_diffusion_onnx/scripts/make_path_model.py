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
Export a tiny ONNX model to verify the OnnxPathModel C++ backend (Nav2 Mode B).

The model maps a goal-aligned context [goal_distance, 0] to a [1, K, H, 2] tensor
of K candidate paths (x, y in the aligned frame). It is not trained: each
candidate is the straight line from (0, 0) to (d, 0) plus a fixed per-candidate
half-sine lateral bow, built analytically inside the graph so the fixture is
deterministic and exercises the C++ tensor plumbing, reshape, and frame
transform without depending on a trained planner.
"""

import sys

import torch
from torch import nn

PATH_K = 5
PATH_H = 12


class TinyPathModel(nn.Module):
    """Build [1, K, H, 2] aligned paths from context [1, 2] = [d, 0]."""

    def __init__(self):
        super().__init__()
        # t in [0, 1] along the path, and a fixed bump shape t*(1-t)*4 (0 at ends).
        t = torch.linspace(0.0, 1.0, PATH_H)
        self.register_buffer('t', t)
        self.register_buffer('bump', t * (1.0 - t) * 4.0)
        # Distinct bow amplitudes per candidate (left/straight/right detours).
        self.register_buffer('amps', torch.linspace(-0.4, 0.4, PATH_K))

    def forward(self, context):
        """Map [1, 2] context to a [1, K, H, 2] aligned path tensor."""
        d = context[:, 0:1]                       # [1, 1] goal distance
        xs = self.t.reshape(1, 1, PATH_H) * d.reshape(-1, 1, 1)      # [1, 1, H]
        xs = xs.expand(-1, PATH_K, -1)                               # [1, K, H]
        bow = self.amps.reshape(1, PATH_K, 1) * self.bump.reshape(1, 1, PATH_H)
        ys = bow * d.reshape(-1, 1, 1)                               # scale with d
        return torch.stack([xs, ys], dim=-1)                        # [1, K, H, 2]


def main():
    """Export the fixture model to the path given as argv[1]."""
    out_path = sys.argv[1]
    model = TinyPathModel().eval()
    dummy = torch.zeros(1, 2)
    torch.onnx.export(
        model, dummy, out_path,
        input_names=['context'], output_names=['paths'],
        opset_version=17)
    print('wrote', out_path)


if __name__ == '__main__':
    main()
