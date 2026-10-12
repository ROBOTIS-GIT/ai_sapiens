#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.
"""Replay offline reference-check CSV in the MuJoCo viewer (requires Python mujoco/numpy)."""

import argparse
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--rollout', required=True)
    args = parser.parse_args()
    import mujoco
    import mujoco.viewer
    import numpy as np

    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    rows = np.atleast_2d(np.loadtxt(args.rollout, delimiter=',', skiprows=1))
    if rows.shape[1] != model.nq + 1 or not np.isfinite(rows).all():
        raise ValueError('Rollout does not match scene qpos dimensions')
    if len(rows) < 2 or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('Rollout timestamps must increase')
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 2.5
        while viewer.is_running():
            started = time.monotonic()
            for row in rows:
                if not viewer.is_running():
                    return
                delay = started + row[0] - rows[0, 0] - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                with viewer.lock():
                    data.qpos[:] = row[1:]
                    mujoco.mj_forward(model, data)
                    viewer.cam.lookat[:] = data.qpos[:3]
                viewer.sync()


if __name__ == '__main__':
    main()
