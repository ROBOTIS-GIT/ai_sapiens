#!/usr/bin/env python3
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

"""Validate and copy a Cyclo K1 controller-moe export into the local policy assets."""

import argparse
import hashlib
from pathlib import Path
import shutil

import numpy as np
import onnxruntime as ort
import yaml


OBSERVATIONS = {
    'motion_command': 46,
    'motion_anchor_ori_b': 6,
    'base_ang_vel': 3,
    'joint_pos_rel': 23,
    'joint_vel_rel': 23,
    'last_action': 23,
    'robot_root_position_xy_w': 2,
    'reference_root_position_xy_w': 2,
    'velocity_commands': 3,
}

OBSERVATION_SCHEMA = 'mimic_global_position_v1'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate(run, motion, joint_order):
    """Check the export against the existing K1 mimic observation/action contract."""
    config = yaml.safe_load((run / 'params/sim2real.yaml').read_text())
    # Training YAML contains Python tags; BaseLoader reads them only as data.
    agent = yaml.load((run / 'params/agent.yaml').read_text(), Loader=yaml.BaseLoader)
    env = yaml.load((run / 'params/env.yaml').read_text(), Loader=yaml.BaseLoader)
    require(agent['actor']['class_name'].endswith(':ResidualMoEModel'), 'Not a MoE actor')
    require(agent['actor']['obs_normalization'] == 'false', 'MoE normalization must be off')
    require(config['policy_joints'] == joint_order, 'Policy and K1 CSV joint orders differ')
    require(abs(config['step_dt'] - 0.02) < 1e-8, 'Expected 50 Hz policy / 50 FPS motion')
    reference = config['commands']['reference_trajectory']
    training_reference = env['commands']['reference_trajectory']
    require(list(env['observations']['actor']['terms']) == list(OBSERVATIONS),
            'Training actor must use global position observations; '
            'do not re-export a velocity-trained checkpoint as a position policy')
    require(reference['observation_origin'] == 'episode', 'Expected episode XY origin')
    require(training_reference['observation_origin'] == 'episode', 'Training XY origin differs')
    require(Path(training_reference['root_position_csv_file']).name == motion.name,
            'CSV filename differs from training configuration')
    steering = reference['steering']
    require(steering.get('tracking_mode', 'trajectory') == 'trajectory' and
            training_reference['steering'].get('tracking_mode', 'trajectory') == 'trajectory',
            'Position MoE requires integrated trajectory steering in export and training')
    require('velocity_estimator_time_constant' not in steering and
            'velocity_estimator_time_constant' not in training_reference['steering'],
            'Velocity estimator settings belong to an incompatible training schema')
    for key in ('lin_vel_x', 'lin_vel_y', 'yaw_rate'):
        values = steering[key]
        require(len(values) == 2 and np.isfinite(values).all() and
                values[0] <= 0 <= values[1], f'Invalid steering range: {key}')
        require(values == [float(v) for v in training_reference['steering'][key]],
                f'Steering range differs from training: {key}')
    tau = steering['smoothing_time_constant']
    require(np.isfinite(tau) and tau >= 0 and
            tau == float(training_reference['steering']['smoothing_time_constant']),
            'Steering smoothing differs from training')
    observations = config['observations']
    require(list(observations) == list(OBSERVATIONS), 'Unexpected actor observation order')
    for name, size in OBSERVATIONS.items():
        term = observations[name]
        require(term['history_length'] == 1 and term['scale'] == [1.0] * size and
                term['clip'] is None, f'Unexpected observation configuration: {name}')
    actions = config['actions']['joint_pos']
    for key in ('scale', 'offset'):
        require(len(actions[key]) == 23 and np.isfinite(actions[key]).all(),
                f'Invalid action {key}')
    require(actions.get('clip') is None, 'Unexpected processed action clipping')
    limit = float(agent['clip_actions'])
    require(limit > 0 and np.isfinite(limit) and
            actions['raw_clip'] == [[-limit, limit]] * 23, 'Training action clip differs')
    frames = np.loadtxt(motion, delimiter=',', ndmin=2)
    require(frames.shape[0] > 1 and frames.shape[1] == 30 and np.isfinite(frames).all(),
            'Expected finite MJLab CSV rows: XYZ, quaternion XYZW, 23 joint positions')

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(run / 'exported/policy.onnx'), options,
                                   providers=['CPUExecutionProvider'])
    inputs, outputs = session.get_inputs(), session.get_outputs()
    require(len(inputs) == 1 and inputs[0].shape == [1, 131] and
            inputs[0].type == 'tensor(float)', 'Expected float32 ONNX input [1, 131]')
    require(len(outputs) == 1 and outputs[0].shape == [1, 23] and
            outputs[0].type == 'tensor(float)', 'Expected float32 ONNX output [1, 23]')
    metadata = session.get_modelmeta().custom_metadata_map
    require(metadata['run_path'] == run.name, 'ONNX belongs to another training run')
    require(metadata['joint_names'].split(',') == joint_order, 'ONNX joint order differs')
    require(metadata['observation_names'].split(',') == list(OBSERVATIONS),
            'ONNX observation order differs')
    return frames.shape[0]


def main():
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', required=True, type=Path,
                        help='Training run with exported/policy.onnx and params/*.yaml')
    parser.add_argument('--motion-csv', required=True, type=Path)
    parser.add_argument('--output', type=Path,
                        default=package / 'assets/k1/mimic/glopodanamite_controller_moe')
    args = parser.parse_args()
    run = args.run_dir.resolve()
    motion = args.motion_csv.resolve()
    destination = args.output.resolve()
    root_config = yaml.safe_load((package / 'config/k1_config.yaml').read_text())
    joint_order = root_config['robot_joint_order']
    frame_count = validate(run, motion, joint_order)
    files = {Path('exported/policy.onnx'): run / 'exported/policy.onnx',
             Path('params') / motion.name: motion}
    for name in ('sim2real.yaml', 'env.yaml', 'agent.yaml'):
        files[Path('params') / name] = run / 'params' / name
    # Import into a new directory so existing bundles are never replaced implicitly.
    require(not destination.exists(), f'Output already exists: {destination}')
    hashes = {str(relative): hashlib.sha256(source.read_bytes()).hexdigest()
              for relative, source in files.items()}
    manifest = {
        'task': 'Cyclo-Mimic-K1-Rev1-Dynamite-Gloposition-controller-moe',
        'observation_schema': OBSERVATION_SCHEMA,
        'source_run': str(run), 'source_motion': str(motion),
        'motion_frames': frame_count, 'sha256': hashes,
    }
    destination.mkdir(parents=True)
    for relative, source in files.items():
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    (destination / 'source_manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    print(f'Imported MoE bundle: {destination} '
          f'(131 observations, 23 actions, {frame_count} frames)')


if __name__ == '__main__':
    main()
