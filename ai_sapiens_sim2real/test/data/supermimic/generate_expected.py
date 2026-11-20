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
Generate synthetic deployment fixtures with the training exporter and goal extractor.

Run with cyclo_mjlab's Python environment:
  python generate_expected.py /path/to/cyclo_mjlab
No learned weights or robot execution are involved.
"""

import os
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace


def main():
    import numpy as np
    import torch
    import yaml
    import onnx
    from scipy.spatial.transform import Rotation
    from supermimic.representations import create_representation
    from supermimic.tasks.task_spec import resolve_task_specs
    from supermimic.runners.supermimic_runner import SuperMimicRunnerCfg
    from source.tasks.supermimic.export import (
        PolicyBundle,
        policy_contract,
        pack_input,
        motion_goal,
    )
    from source.tasks.supermimic.export_sim2real_cfg import export_sim2real_cfg
    from source.tasks.supermimic.config.k1_rev1.base_env_cfg import make_env_cfg

    folder = Path(__file__).parent
    cfg = make_env_cfg()
    rcfg = SuperMimicRunnerCfg(representation='gencoord', actor_obs='proprio', history_length=3)
    action = {
        'joint_order': ['left_hip_roll_joint', 'right_hip_roll_joint'],
        'q0': [0.1, -0.2],
        'qdot0': [0.0, 0.0],
        'scale': [0.5, 0.5],
        'offset': [0.1, -0.2],
        'imu_site_body': 'pelvis',
        'imu_site_quaternion_wxyz': [1.0, 0.0, 0.0, 0.0],
    }
    cfg.scene.entities['robot'].init_state.joint_pos = dict(
        zip(action['joint_order'], action['q0'])
    )
    cfg.actions['joint_pos'].scale = 0.5
    cfg.actions['joint_pos'].use_default_offset = True
    rep = create_representation(
        'gencoord',
        17,
        2,
        0.02,
        cfg.decimation,
        torch.tensor(action['q0']),
        actor_obs_mode='proprio',
    )
    specs = resolve_task_specs(rcfg.tasks, rep, 130, 'cpu')
    actor = SimpleNamespace(
        obs_dim=12,
        task_specs=specs,
        obs_normalizer=SimpleNamespace(eps=1e-8),
        goal_normalizers={s.name: SimpleNamespace(eps=1e-8) for s in specs},
    )
    bundle = PolicyBundle(
        actor, rep, rcfg, cfg, {'task_id': 'synthetic', 'robot': {}}, action, [3, 3, 2, 2, 2]
    )
    for encoder in ('motion', 'velocity'):
        contract = policy_contract(bundle, encoder)
        name = '' if encoder == 'motion' else 'velocity/'
        dest = folder / name
        dest.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory() as tmp:
            yaml_path = export_sim2real_cfg(
                tmp, cfg, rcfg, rep, action['joint_order'], encoder=encoder
            )
            shutil.copyfile(yaml_path, dest / 'sim2real.yaml')
        h = onnx.helper
        initializers = [
            onnx.numpy_helper.from_array(np.array(v, dtype=np.int64), k)
            for k, v in (('start', [22]), ('end', [24]), ('axis', [1]))
        ]
        graph = h.make_graph(
            [h.make_node('Slice', ['obs', 'start', 'end', 'axis'], ['actions'])],
            'synthetic_latest_joint_position',
            [h.make_tensor_value_info('obs', 1, contract['input']['shape'])],
            [h.make_tensor_value_info('actions', 1, [1, 2])],
            initializers,
        )
        model = h.make_model(graph, opset_imports=[h.make_opsetid('', 13)], ir_version=8)
        # Deployment must also work with models that have no custom metadata.
        onnx.checker.check_model(model)
        onnx.save(model, dest / 'policy.onnx')

    t = np.arange(64, dtype=np.float32) * 0.02
    xyz = np.column_stack((t * t, t * 0.2, 0.7 + t * 0.01))
    quat = Rotation.from_euler(
        'xyz', np.column_stack((0.1 + t * 0.03, -0.15 + t * 0.04, 0.6 + t * 0.5))
    ).as_quat()
    q = np.column_stack((t * t + 0.1, t * 0.3 - 0.2))
    values = np.column_stack((xyz, quat, q)).astype(np.float32)
    np.savetxt(
        folder / 'motion.csv',
        values,
        delimiter=',',
        header=(
            'root_pos_x,root_pos_y,root_pos_z,root_rot_x,root_rot_y,root_rot_z,root_rot_w,'
            'left_hip_roll_joint,right_hip_roll_joint'
        ),
        comments='',
        fmt='%.9g',
    )
    qd = np.gradient(values[:, 7:], 0.02, axis=0)
    motion = SimpleNamespace(
        joint_pos=torch.tensor(values[:, 7:]),
        joint_vel=torch.tensor(qd),
        root_pos_w=torch.tensor(values[:, :3]),
        root_quat_w=torch.tensor(values[:, [6, 3, 4, 5]]),
        motion_ends=torch.tensor([len(t)]),
    )
    first = Rotation.from_euler('xyz', [-0.1, 0.2, -0.4]).as_matrix()
    delta = np.arctan2(
        Rotation.from_quat(values[0, 3:7]).as_matrix()[1, 0],
        Rotation.from_quat(values[0, 3:7]).as_matrix()[0, 0],
    ) - np.arctan2(first[1, 0], first[0, 0])
    alignment = Rotation.from_euler('z', delta).as_matrix()
    history = None
    expected = []
    for i in range(len(t)):
        robot = Rotation.from_euler('xyz', [-0.1, 0.2, -0.4 + i * 0.002]).as_matrix()
        sample = torch.tensor(
            np.r_[
                -robot[2],
                [0.1, 0.2, 0.3],
                [0.3 + i * 0.001, -0.5],
                [0.8, -0.9],
                [i * 0.002, -i * 0.001],
            ],
            dtype=torch.float32,
        )
        history = (
            sample.repeat(3, 1) if history is None else torch.cat((history[1:], sample[None]))
        )
        aligned_quat = Rotation.from_matrix(alignment @ robot).as_quat()[[3, 0, 1, 2]]
        pose = torch.tensor(np.r_[np.zeros(3), aligned_quat], dtype=torch.float32)[None]
        goal = motion_goal(bundle, motion, torch.tensor([i + 1]), pose)
        expected.append(pack_input(history[None], goal, bundle.term_widths)[0].tolist())
    (folder / 'expected.yaml').write_text(yaml.safe_dump({'observations': expected}))


if __name__ == '__main__':
    os.environ['TORCHDYNAMO_DISABLE'] = '1'
    sys.path.insert(0, sys.argv[1])
    main()
