#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.
"""Generate a native-runtime oracle using actual training observations and Torch policy.

Covers both history windows, next-frame references, residual targets and reentry
under a different odometry origin/yaw. Run in humanoid_motion's Python environment.
"""

import argparse
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--humanoid-motion', type=Path, required=True)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    sys.path.insert(0, str(args.humanoid_motion.resolve()))
    import mujoco
    import torch
    from source.tasks.any2track.config import ASSETS, CONSTANTS
    from source.tasks.any2track.env import Any2TrackEnv
    from source.tasks.any2track.export import load_export_checkpoint
    from source.tasks.any2track.motion import load_motion
    from source.tasks.any2track.rl.velocity import feature_indices
    from source.tasks.any2track import math as qm

    torch.set_num_threads(1)
    checkpoint, cfg, env_cfg, policy = load_export_checkpoint(args.checkpoint)
    policy.eval()
    model = mujoco.MjModel.from_xml_path(str(ASSETS / 'robotis_k1/scene_mjx_flat_terrain.xml'))
    data = mujoco.MjData(model)
    ref = load_motion(cfg.motion_file, model, 0.02)

    def tensor(x):
        return torch.tensor(np.array(x), dtype=torch.float32)[None]

    features = feature_indices(env_cfg['obs_keys'])
    fake = SimpleNamespace(
        num_envs=1,
        device='cpu',
        cfg=env_cfg,
        stage='adapter',
        noise=False,
        default=torch.tensor(CONSTANTS['DEFAULT_QPOS'][7:]),
        ids={
            'imu': model.site('imu_in_pelvis').id,
            'feet_sites': [model.site(n).id for n in CONSTANTS['FEET_ALL_SITES']],
            'pelvis': model.body('pelvis').id,
        },
        contacts=lambda: torch.zeros(1, 2),
        history=torch.zeros(1, 79, 75),
        data=lambda: {key: tensor(getattr(data, key)) for key in ref},
        sensor=lambda name: tensor(
            data.sensordata[
                model.sensor(name).adr[0]:model.sensor(name).adr[0]
                + model.sensor(name).dim[0]
            ]
        ),
    )
    records = []
    for episode in range(2):
        vh = None
        previous = None
        shift = np.array([2.0, -3.0]) if episode else np.zeros(2)
        angle = 1.1 if episode else 0.0
        world_yaw = qm.from_rpy(torch.tensor([[0.0, 0.0, angle]]))
        rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        for tick in range(85):
            data.qpos[:] = ref['qpos'][tick]
            data.qvel[:] = ref['qvel'][tick]
            # Nontrivial sensor/pose changes, identical between coordinate systems.
            data.qpos[0] += 0.002 * tick
            data.qpos[1] -= 0.003 * tick
            data.qpos[7:] += 0.04 * np.sin(np.arange(23) + tick * 0.1) * (tick > 0)
            data.qvel[6:] += 0.2 * np.cos(np.arange(23) + tick * 0.2)
            if tick:
                q = qm.mul(
                    tensor(data.qpos[3:7]),
                    qm.from_rpy(torch.tensor([[0.1, -0.07, 0.003 * tick]])),
                )
                data.qpos[3:7] = q.numpy()[0]
            mujoco.mj_forward(model, data)
            upcoming = {key: tensor(value[tick + 1]) for key, value in ref.items()}
            fake.reference = lambda: upcoming
            if tick == 0:
                fake.targets = tensor(data.qpos[7:])
                _, current = Any2TrackEnv.observations(fake)
                fake.history = torch.cat((current, fake.targets), -1)[:, None].repeat(1, 79, 1)
            else:
                fake.targets = previous
            obs, current = Any2TrackEnv.observations(fake)
            sensor_frame = obs['state'][:, features]
            vh = (
                sensor_frame[:, None].repeat(1, 20, 1)
                if vh is None
                else torch.cat((vh[:, 1:], sensor_frame[:, None]), 1)
            )
            ah = fake.history.transpose(1, 2).contiguous()
            with torch.no_grad():
                action, _ = policy(obs['state'], vh, ah)
            target = upcoming['qpos'][:, 7:] + action * torch.tensor(env_cfg['action_scale'])
            raw_orientation = qm.mul(world_yaw, tensor(data.qpos[3:7])).numpy()[0]
            raw_xy = rot @ data.qpos[:2] + shift
            sample = np.concatenate(
                (raw_xy, raw_orientation, data.qvel[3:6], data.qpos[7:], data.qvel[6:])
            )
            record = np.concatenate(
                (
                    sample,
                    obs['state'].numpy().ravel(),
                    vh.numpy().ravel(),
                    ah.numpy().ravel(),
                    action.numpy().ravel(),
                    target.numpy().ravel(),
                )
            )
            assert len(record) == 7660
            records.append(
                struct.pack('<II', int(tick == 0), episode) + record.astype('<f4').tobytes()
            )
            # Training pairs pre-action sensor features with the resulting motor target.
            fake.history = torch.cat(
                (fake.history[:, 1:], torch.cat((current, target), -1)[:, None]), 1
            )
            previous = target
    args.output.write_bytes(b'A2TPAR01' + struct.pack('<I', len(records)) + b''.join(records))
    print(f'Wrote {len(records)} training/Torch oracle steps to {args.output}')


if __name__ == '__main__':
    main()
