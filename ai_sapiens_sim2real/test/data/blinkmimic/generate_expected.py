# Copyright 2026 ROBOTIS CO., LTD.
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
Regenerate CPU parity vectors against the pinned upstream Blink MotionLib.

Usage: python generate_expected.py /path/to/cyclo_mjlab/third_party/blinkmimic/src
Upstream: fuyichen0725/blinkmimic d264658d6c9ce61fb52f9f0f8985a369f3ccc409.
The two-joint CSV and sensor samples are synthetic, not trained robot assets.
"""

from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yaml


class RootKinematics:
    def forward(self, p, rotation, v, w, q, qd):
        return p[:, None], rotation[:, None], v[:, None], w[:, None]


def load_motion(path):
    values = np.loadtxt(path, delimiter=',', skiprows=1, dtype=np.float32)
    return values[:, :3], values[:, [6, 3, 4, 5]], values[:, 7:], 50


def main():
    from blinkmimic.motion import MotionLib
    from blinkmimic.rotations import mat_to_6d, quat_to_mat

    folder = Path(__file__).parent
    motion = MotionLib([str(folder / 'motion.csv')], RootKinematics(), 'cpu', load_motion)
    q = Rotation.from_euler('xyz', [-0.1, 0.2, -0.4]).as_quat()[[3, 0, 1, 2]]
    robot = quat_to_mat(torch.tensor(q, dtype=torch.float32))
    delta = torch.atan2(motion.root_R[0, 1, 0], motion.root_R[0, 0, 0]) - torch.atan2(
        robot[1, 0], robot[0, 0]
    )
    alignment = torch.tensor(
        [[delta.cos(), -delta.sin(), 0], [delta.sin(), delta.cos(), 0], [0, 0, 1]]
    )
    history = None
    expected = []
    for i in range(motion.num_frames):
        relative = mat_to_6d((alignment @ robot).T @ motion.root_R[i])
        sensor = torch.cat(
            (
                torch.tensor([0.4, -0.7]) - torch.tensor([0.1, -0.2]),
                torch.tensor([0.8, -0.9, 0.1, 0.2, 0.3]),
                -robot[2],
                torch.zeros(2),
                relative,
            )
        )
        history = (
            sensor.repeat(3, 1) if history is None else torch.cat((history[1:], sensor[None]))
        )
        goal = motion.goal(torch.tensor([i]), 2)[0]
        expected.append(torch.cat((history.flatten(), goal)).tolist())
    (folder / 'expected.yaml').write_text(yaml.safe_dump({'observations': expected}))


if __name__ == '__main__':
    sys.path.insert(0, sys.argv[1])
    main()
