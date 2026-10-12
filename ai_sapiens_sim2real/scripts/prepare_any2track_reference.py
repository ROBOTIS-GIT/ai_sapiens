#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.
"""Prepare exact training reference FK for the native Any2Track runtime.

Run in the humanoid_motion Python environment. Original ONNX/YAML/CSV stay intact.
The sidecar binds all three files and training-derived reference data by SHA256.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import sys

import numpy as np
import yaml


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--humanoid-motion', type=Path, required=True)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--motion', type=Path, help='Training NPZ (default: Redred)')
    args = parser.parse_args()
    root = args.humanoid_motion.resolve()
    sys.path.insert(0, str(root))
    import mujoco
    import onnx
    from source.tasks.any2track.motion import load_motion

    assets = root / 'source/tasks/any2track/assets'
    params = args.bundle.resolve() / 'params'
    config = yaml.safe_load((params / 'sim2real.yaml').read_text())
    if config['runtime_type'] != 'any2track_gt_xy_adapter':
        raise ValueError('Expected any2track_gt_xy_adapter')
    constants = json.loads((assets / 'constants.json').read_text())
    if config['policy_joints'] != constants['ACTION_JOINT_NAMES']:
        raise ValueError('Training joint order differs from deployment YAML')
    model = mujoco.MjModel.from_xml_path(str(assets / 'robotis_k1/scene_mjx_flat_terrain.xml'))
    motion = args.motion or root / 'source/assets/motions/K1_rev1/redred/redred_k1.npz'
    ref = load_motion(motion, model, config['step_dt'])
    csv = params / 'redred_k1.csv'
    supplied = np.loadtxt(csv, delimiter=',')
    # User CSV uses XYZ + XYZW + joints; training/generalized coordinates use WXYZ.
    expected = ref['qpos'][:, [0, 1, 2, 4, 5, 6, 3, *range(7, 30)]]
    if supplied.shape != expected.shape or not np.allclose(
        supplied, expected, atol=2e-6, rtol=0
    ):
        raise ValueError(
            'Supplied CSV is not the selected training motion; refusing to bind assets'
        )
    policy_path = args.bundle.resolve() / 'exported/policy.onnx'
    onnx_model = onnx.load(str(policy_path))
    meta = {p.key: p.value for p in onnx_model.metadata_props}
    if meta.get('runtime_type') != config['runtime_type']:
        raise ValueError('ONNX and YAML runtime types disagree')
    sites = [model.site(n).id for n in constants['FEET_ALL_SITES']]
    frames = np.concatenate((ref['qpos'], ref['qvel'], ref['site_xpos'][:, sites, 2]), axis=1)
    if frames.shape[1] != 63 or not np.isfinite(frames).all():
        raise ValueError('Invalid reference frames')
    binary = params / 'any2track_reference.bin'
    binary.write_bytes(
        b'A2TREF01' + struct.pack('<I', len(frames)) + frames.astype('<f4').tobytes()
    )
    manifest = {
        'version': 1,
        'columns': 63,
        'frames': len(frames),
        'step_dt': config['step_dt'],
        'joint_names': config['policy_joints'],
        'foot_sites': constants['FEET_ALL_SITES'],
        'reference_file': binary.name,
        'reference_sha256': sha256(binary),
        'motion_csv_sha256': sha256(csv),
        'policy_sha256': sha256(policy_path),
        'sim2real_sha256': sha256(params / 'sim2real.yaml'),
        'training_motion_sha256': sha256(motion),
        'training_checkpoint': meta.get('checkpoint'),
        'training_iteration': meta.get('iteration'),
        'reference_model': 'humanoid_motion/source/tasks/any2track/assets/robotis_k1',
        'reference_model_sha256': sha256(assets / 'robotis_k1/k1_mjx.xml'),
    }
    (params / 'any2track_reference.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    duration = (len(frames) - 1) * config['step_dt']
    print(f'Prepared {len(frames)} frames ({duration:.2f} s) in {params}')
    print(f'Model iteration {meta.get("iteration")}; CSV matches training NPZ')


if __name__ == '__main__':
    main()
