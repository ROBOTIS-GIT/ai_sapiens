"""Regenerate CPU parity vectors against the pinned upstream Blink MotionLib.

Usage: python generate_expected.py /path/to/cyclo_mjlab/third_party/blinkmimic/src
Upstream: fuyichen0725/blinkmimic d264658d6c9ce61fb52f9f0f8985a369f3ccc409.
The two-joint CSV and sensor samples are synthetic, not trained robot assets.
"""

import sys
from pathlib import Path

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, sys.argv[1])
from blinkmimic.motion import MotionLib
from blinkmimic.rotations import mat_to_6d, quat_to_mat


class RootKinematics:
    def forward(self, p, rotation, v, w, q, qd):
        return p[:, None], rotation[:, None], v[:, None], w[:, None]


def load_motion(path):
    values = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.float32)
    return values[:, :3], values[:, [6, 3, 4, 5]], values[:, 7:], 50


folder = Path(__file__).parent
motion = MotionLib([str(folder / "motion.csv")], RootKinematics(), "cpu", load_motion)
q = Rotation.from_euler("xyz", [-.1, .2, -.4]).as_quat()[[3, 0, 1, 2]]
robot = quat_to_mat(torch.tensor(q, dtype=torch.float32))
delta = torch.atan2(motion.root_R[0, 1, 0], motion.root_R[0, 0, 0]) - torch.atan2(robot[1, 0], robot[0, 0])
alignment = torch.tensor([[delta.cos(), -delta.sin(), 0], [delta.sin(), delta.cos(), 0], [0, 0, 1]])
history = None
expected = []
for i in range(motion.num_frames):
    relative = mat_to_6d((alignment @ robot).T @ motion.root_R[i])
    sensor = torch.cat((torch.tensor([.4, -.7]) - torch.tensor([.1, -.2]),
                        torch.tensor([.8, -.9, .1, .2, .3]), -robot[2], torch.zeros(2), relative))
    history = sensor.repeat(3, 1) if history is None else torch.cat((history[1:], sensor[None]))
    goal = motion.goal(torch.tensor([i]), 2)[0]
    expected.append(torch.cat((history.flatten(), goal)).tolist())
(folder / "expected.yaml").write_text(yaml.safe_dump({"observations": expected}))
