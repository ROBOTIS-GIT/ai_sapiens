"""Generate synthetic deployment fixtures with the training exporter and goal extractor.

Run with cyclo_mjlab's Python environment:
  python generate_expected.py /path/to/cyclo_mjlab
No learned weights or robot execution are involved.
"""
import os
import sys
import shutil
from pathlib import Path
from types import SimpleNamespace
os.environ["TORCHDYNAMO_DISABLE"] = "1"
sys.path.insert(0, sys.argv[1])
import numpy as np
import torch
import yaml
import onnx
from scipy.spatial.transform import Rotation
from supermimic.representations import create_representation
from supermimic.tasks.task_spec import resolve_task_specs
from supermimic.runners.supermimic_runner import SuperMimicRunnerCfg
from source.tasks.supermimic.export import PolicyBundle, policy_contract, pack_input, motion_goal
from source.tasks.supermimic.export_sim2real_cfg import export_sim2real_cfg
from source.tasks.supermimic.config.k1_rev1.base_env_cfg import make_env_cfg

folder = Path(__file__).parent
cfg = make_env_cfg()
rcfg = SuperMimicRunnerCfg(representation="gencoord", actor_obs="proprio", history_length=3)
action = dict(joint_order=["left_hip_roll_joint", "right_hip_roll_joint"],
              q0=[.1, -.2], qdot0=[0., 0.], scale=[.5, .5], offset=[.1, -.2],
              imu_site_body="pelvis", imu_site_quaternion_wxyz=[1., 0., 0., 0.])
cfg.scene.entities["robot"].init_state.joint_pos = dict(zip(action["joint_order"], action["q0"]))
cfg.actions["joint_pos"].scale = .5
cfg.actions["joint_pos"].use_default_offset = True
rep = create_representation("gencoord", 17, 2, .02, cfg.decimation,
                            torch.tensor(action["q0"]), actor_obs_mode="proprio")
specs = resolve_task_specs(rcfg.tasks, rep, 130, "cpu")
actor = SimpleNamespace(obs_dim=12, task_specs=specs,
                        obs_normalizer=SimpleNamespace(eps=1e-8),
                        goal_normalizers={s.name: SimpleNamespace(eps=1e-8) for s in specs})
bundle = PolicyBundle(actor, rep, rcfg, cfg, {"task_id": "synthetic", "robot": {}},
                       action, [3, 3, 2, 2, 2])
for encoder in ("motion", "velocity"):
    contract = policy_contract(bundle, encoder)
    name = "" if encoder == "motion" else "velocity/"
    dest = folder / name
    dest.mkdir(exist_ok=True)
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = export_sim2real_cfg(tmp, cfg, rcfg, rep, action["joint_order"], encoder=encoder)
        shutil.copyfile(yaml_path, dest / "sim2real.yaml")
    h = onnx.helper
    initializers = [onnx.numpy_helper.from_array(np.array(v, dtype=np.int64), k)
                    for k, v in (("start", [22]), ("end", [24]), ("axis", [1]))]
    graph = h.make_graph([h.make_node("Slice", ["obs", "start", "end", "axis"], ["actions"])],
                        "synthetic_latest_joint_position", [h.make_tensor_value_info("obs", 1, contract["input"]["shape"])],
                        [h.make_tensor_value_info("actions", 1, [1, 2])], initializers)
    model = h.make_model(graph, opset_imports=[h.make_opsetid("", 13)], ir_version=8)
    # Deployment must also work with models that have no custom metadata.
    onnx.checker.check_model(model)
    onnx.save(model, dest / "policy.onnx")

t = np.arange(64, dtype=np.float32) * .02
xyz = np.column_stack((t*t, t*.2, .7+t*.01))
quat = Rotation.from_euler("xyz", np.column_stack((.1+t*.03, -.15+t*.04, .6+t*.5))).as_quat()
q = np.column_stack((t*t+.1, t*.3-.2))
values = np.column_stack((xyz, quat, q)).astype(np.float32)
np.savetxt(folder / "motion.csv", values, delimiter=",", header="root_pos_x,root_pos_y,root_pos_z,root_rot_x,root_rot_y,root_rot_z,root_rot_w,left_hip_roll_joint,right_hip_roll_joint", comments="", fmt="%.9g")
qd = np.gradient(values[:, 7:], .02, axis=0)
motion = SimpleNamespace(joint_pos=torch.tensor(values[:,7:]), joint_vel=torch.tensor(qd),
                         root_pos_w=torch.tensor(values[:,:3]), root_quat_w=torch.tensor(values[:,[6,3,4,5]]),
                         motion_ends=torch.tensor([len(t)]))
first = Rotation.from_euler("xyz", [-.1,.2,-.4]).as_matrix()
delta = np.arctan2(Rotation.from_quat(values[0,3:7]).as_matrix()[1,0], Rotation.from_quat(values[0,3:7]).as_matrix()[0,0]) - np.arctan2(first[1,0], first[0,0])
alignment = Rotation.from_euler("z", delta).as_matrix()
history = None
expected = []
for i in range(len(t)):
    robot = Rotation.from_euler("xyz", [-.1,.2,-.4+i*.002]).as_matrix()
    sample = torch.tensor(np.r_[-robot[2], [.1,.2,.3], [.3+i*.001,-.5], [.8,-.9], [i*.002,-i*.001]], dtype=torch.float32)
    history = sample.repeat(3,1) if history is None else torch.cat((history[1:],sample[None]))
    aligned_quat = Rotation.from_matrix(alignment @ robot).as_quat()[[3,0,1,2]]
    pose = torch.tensor(np.r_[np.zeros(3),aligned_quat],dtype=torch.float32)[None]
    goal = motion_goal(bundle, motion, torch.tensor([i+1]), pose)
    expected.append(pack_input(history[None],goal,bundle.term_widths)[0].tolist())
(folder / "expected.yaml").write_text(yaml.safe_dump({"observations":expected}))
