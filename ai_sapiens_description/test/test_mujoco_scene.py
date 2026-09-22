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
#
# Author: Kiwoong Park

import pathlib

import pytest

mujoco = pytest.importorskip('mujoco')

PKG_DIR = pathlib.Path(__file__).resolve().parents[1]


def load(scene_name):
    return mujoco.MjModel.from_xml_path(str(PKG_DIR / 'mujoco' / 'k1' / scene_name))


def test_scene_has_imu_sensors():
    model = load('scene.xml')
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'imu') >= 0
    for sensor in ('imu_quat', 'imu_gyro', 'imu_acc'):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor) >= 0


@pytest.mark.parametrize('scene', ['scene.xml', 'scene_gantry.xml'])
def test_k1_mimic_physics_matches_training(scene):
    model = load(scene)
    assert model.opt.timestep == pytest.approx(0.005)
    assert model.opt.iterations == 10
    assert model.opt.ls_iterations == 20
    assert model.opt.ccd_iterations == 50
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    feet = 0
    for index in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or ''
        if '_collision_' not in name:
            continue
        assert model.geom_contype[index] == model.geom_conaffinity[index] == 1
        if '_ankle_roll_link_collision_' in name:
            feet += 1
            assert model.geom_condim[index] == 3
            assert model.geom_priority[index] == 1
            assert model.geom_friction[index, 0] == pytest.approx(0.6)
        else:
            assert model.geom_condim[index] == 1
    assert feet == 18


def test_gantry_scene():
    model = load('scene_gantry.xml')
    gantry_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gantry')
    assert gantry_body >= 0
    assert model.body_mocapid[gantry_body] >= 0
    eq_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, 'gantry_weld')
    assert eq_id >= 0
    # Weld must be active at spawn and satisfied at qpos0 (no initial jump).
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.eq_active[eq_id] == 1


def test_hanging_robot_settles_above_ground():
    model = load('scene_gantry.xml')
    data = mujoco.MjData(model)
    # Simulate the hang: raise base and mocap by the same offset (Task 4 logic).
    dz = 0.90 - 0.7955
    data.qpos[2] += dz
    data.mocap_pos[0][2] += dz
    for _ in range(round(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    # Base stays hanging near 0.90 m; feet never touch (standing height is 0.7955).
    assert data.qpos[2] > 0.85
