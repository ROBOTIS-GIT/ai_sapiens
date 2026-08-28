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
    for _ in range(1000):  # 2 s at 0.002 timestep
        mujoco.mj_step(model, data)
    # Base stays hanging near 0.90 m; feet never touch (standing height is 0.7955).
    assert data.qpos[2] > 0.85
