#!/usr/bin/env python3
"""
Record the training input generator for the standalone C++ parity probe.

Run in the humanoid_motion Python environment. This creates one evaluation
environment and advances references only; no training/checkpoint is changed.
"""

import argparse
import hashlib
from pathlib import Path
import shutil
import sys

import numpy as np
import torch
import yaml


def validate_reference_config(reference, exported):
    """Reject a live training task that no longer describes the selected export."""
    from source.tasks.mimic.rl.moe import MimicMoERunner
    features = exported.get('required_runtime_features')
    version = {'dance_motion_reference_v2': 4, 'dance_motion_reference_v3': 5}
    if (not isinstance(features, list) or len(features) != 1 or
            version.get(features[0]) != MimicMoERunner.REFERENCE_VERSION):
        raise ValueError('Training reference version differs from the selected export')
    steps = exported['dance_steps']
    if features == ['dance_motion_reference_v3']:
        if (steps.get('release_behavior') != 'finish_active_gesture_no_new_steps' or
                steps.get('step_intent_source') != 'deadbanded_request_before_smoothing'):
            raise ValueError('Unsupported v3 dance release behavior or step intent source')
    for key, value in exported['steering'].items():
        actual = getattr(reference.steering, key)
        if isinstance(value, list):
            actual = list(actual)
        if actual != value:
            raise ValueError(f'Training task steering.{key} differs from the selected export')
    for key, value in exported['dance_steps'].items():
        if key in ('gesture_source', 'scheduling', 'phase_labels', 'phase_labels_sha256',
                   'release_behavior', 'step_intent_source'):
            continue
        if getattr(reference, key) != value:
            raise ValueError(f'Training task dance_steps.{key} differs from the selected export')
    if reference.observation_origin != exported['observation_origin']:
        raise ValueError('Training task observation origin differs from the selected export')
    labels = Path(reference.rhythm_labels_file)
    if hashlib.sha256(labels.read_bytes()).hexdigest() != exported['dance_steps'][
            'phase_labels_sha256']:
        raise ValueError('Training phase labels differ from the selected export')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--training-repo', type=Path, required=True)
    parser.add_argument('--sim2real-config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--device', default='cuda:0')
    args = parser.parse_args()
    sys.path.insert(0, str(args.training_repo.resolve()))
    import source.tasks  # noqa: F401
    from mjlab.envs import ManagerBasedRlEnv
    from mjlab.tasks.registry import load_env_cfg
    exported = yaml.safe_load(args.sim2real_config.read_text())
    cfg = load_env_cfg('Cyclo-Mimic-K1-Rev1-Dynamite-Gloposition-controller-moe', play=True)
    cfg.scene.num_envs = 1
    cfg.events = {}
    cfg.observations['actor'].enable_corruption = False
    ref = cfg.commands['reference_trajectory']
    validate_reference_config(ref, exported['commands']['reference_trajectory'])
    if list(cfg.observations['actor'].terms) != list(exported['observations']):
        raise ValueError('Training observation order differs from the selected export')
    ref.debug_vis = False
    ref.reset_pose_noise = {}
    ref.reset_velocity_noise = {}
    ref.reset_joint_position_noise = (0., 0.)
    env = ManagerBasedRlEnv(cfg, device=args.device)
    inputs, expected = [], []
    scenarios = [
        (0, (0., 0., 0.), (0., 0., 0.)),
        # Both signs, isolated/combined control and release, including a difficult clip section.
        (0, (0.15, 0., 0.), (0., 0., .2)),
        (1500, (-.2, .15, 0.), (0., 0., -.3)),
        (3000, (0., -.3, 0.), (.2, .15, .2)),
        (8500, (.3, 0., .3), (-.15, -.2, -.2)),
    ]
    try:
        if abs(env.step_dt - exported['step_dt']) > 1e-8:
            raise ValueError('Training timestep differs from the selected export')
        args.output.mkdir(parents=True, exist_ok=False)
        shutil.copy2(args.sim2real_config, args.output/'sim2real.yaml')
        shutil.copy2(ref.root_position_csv_file, args.output/'motion.csv')
        print(f'Reference config: {args.sim2real_config.resolve()}', flush=True)
        print('Config SHA256: ' + hashlib.sha256(args.sim2real_config.read_bytes()).hexdigest(),
              flush=True)
        with torch.inference_mode():
            c = env.command_manager.get_term('reference_trajectory')
            for start, first, second in scenarios:
                c._sample_start_frames = lambda ids: c.frame_ids.__setitem__(ids, start)
                c.set_velocity_command([0., 0., 0.])
                env.reset()
                for tick in range(751):
                    request = ([0., 0., 0.] if tick < 30 or 300 <= tick < 410 or tick >= 620
                               else first if tick < 300 else second)
                    c.set_velocity_command(request)
                    if tick:
                        c._update_command()
                    observations = env.observation_manager.compute(update_history=True)
                    obs = observations['actor'][0].cpu().numpy()
                    robot = (c.robot_body_pos_w[0, 0] - env.scene.env_origins[0]).cpu().numpy()
                    quaternion = c.robot_body_quat_w[0, 0].cpu().numpy()
                    inputs.append(np.concatenate((
                        [int(c.frame_ids[0]), int(tick == 0)],
                        request, robot[:2], quaternion, obs)))
                    expected.append(np.concatenate((
                        obs, c.root_shift[0].cpu().numpy(),
                        c.foot_target_pos[0].cpu().numpy().ravel(),
                        c.foot_target_quat[0].cpu().numpy().ravel(),
                        c.joint_target_valid[0, c.kinematics.joint_ids[:, 0]].cpu().numpy(),
                        c.contact_air[0].cpu().numpy(), [int(c.planner.added_steps[0])])))
                print(f'Recorded {start}: 751 frames, '
                      f'added_steps={int(c.planner.added_steps[0])}', flush=True)
    finally:
        env.close()
    np.savetxt(args.output/'input.csv', inputs, delimiter=',', fmt='%.9g')
    np.savetxt(args.output/'expected.csv', expected, delimiter=',', fmt='%.9g')
    print(f'Wrote {len(inputs)} reference frames to {args.output}', flush=True)


if __name__ == '__main__':
    main()
