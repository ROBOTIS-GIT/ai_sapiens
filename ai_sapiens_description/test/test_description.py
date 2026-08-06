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

"""Validate the installed K1 robot-description assets."""

from importlib.util import find_spec
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / 'urdf' / 'k1_rev1' / 'k1.urdf'
XACRO_PATH = PACKAGE_ROOT / 'urdf' / 'k1_rev1' / 'k1.urdf.xacro'
MJCF_PATH = PACKAGE_ROOT / 'mujoco' / 'k1' / 'k1.xml'
SCENE_PATH = PACKAGE_ROOT / 'mujoco' / 'k1' / 'scene.xml'
MESH_ROOT = PACKAGE_ROOT / 'meshes' / 'k1_rev1'

ARMATURE_BY_CLASS = {
    'qc060': 0.00564892,
    'qc060_double_armature': 0.01129784,
    'qc080': 0.01936542,
    'qc080_double_armature': 0.03873084,
}


def _values(text):
    return tuple(float(value) for value in text.split())


def _urdf_model():
    root = ET.parse(URDF_PATH).getroot()
    links = {link.attrib['name']: link for link in root.findall('link')}
    joints = {joint.attrib['name']: joint for joint in root.findall('joint')}
    return root, links, joints


def _is_positive_definite(inertia):
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    minor_2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * iyy * izz
        + 2.0 * ixy * ixz * iyz
        - ixx * iyz * iyz
        - iyy * ixz * ixz
        - izz * ixy * ixy
    )
    return ixx > 0.0 and minor_2 > 0.0 and determinant > 0.0


def _is_qc060_joint(name):
    return any(part in name for part in ('shoulder', 'elbow', 'wrist', 'ankle_roll'))


def _armature_class(name):
    if 'ankle_pitch' in name:
        return 'qc080_double_armature'
    if 'ankle_roll' in name:
        return 'qc060_double_armature'
    return 'qc060' if _is_qc060_joint(name) else 'qc080'


def test_urdf_structure_and_inertias():
    """Ensure the standalone URDF remains a valid physical K1 model."""
    root, links, joints = _urdf_model()
    movable = [joint for joint in joints.values() if joint.attrib['type'] != 'fixed']
    child_links = {joint.find('child').attrib['link'] for joint in joints.values()}

    assert root.attrib['name'] == 'k1'
    assert len(links) == 25
    assert len(joints) == 24
    assert len(movable) == 23
    assert set(links) - child_links == {'pelvis'}
    assert not root.findall('ros2_control')

    for link in links.values():
        inertial = link.find('inertial')
        assert inertial is not None
        assert float(inertial.find('mass').attrib['value']) > 0.0

        values = inertial.find('inertia').attrib
        inertia = tuple(
            float(values[key])
            for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')
        )
        assert _is_positive_definite(inertia)
        assert inertia[0] <= inertia[1] + inertia[2]
        assert inertia[1] <= inertia[0] + inertia[2]
        assert inertia[2] <= inertia[0] + inertia[1]

    for mesh in root.findall('.//mesh'):
        uri = mesh.attrib['filename']
        prefix = 'package://ai_sapiens_description/meshes/k1_rev1/'
        assert uri.startswith(prefix)
        assert (MESH_ROOT / uri.removeprefix(prefix)).is_file()


def test_xacro_expands_with_mock_hardware():
    """Ensure the runtime Xacro expands without any Gazebo simulation path."""
    document = xacro.process_file(
        str(XACRO_PATH),
        mappings={
            'use_mock_hardware': 'true',
            'mock_sensor_commands': 'true',
        },
    )
    root = ET.fromstring(document.toxml())
    controls = root.findall('ros2_control')

    assert len(controls) == 6
    assert len(root.findall('.//plugin')) == 6
    assert {
        plugin.text for plugin in root.findall('.//plugin')
    } == {'mock_components/GenericSystem'}


def test_xacro_expands_with_mujoco_hardware():
    """Ensure the common runtime Xacro selects only the MuJoCo system."""
    document = xacro.process_file(
        str(XACRO_PATH),
        mappings={
            'sim_mujoco': 'true',
            'mujoco_viewer': 'false',
            'mujoco_gantry': 'true',
        },
    )
    root = ET.fromstring(document.toxml())
    controls = root.findall('ros2_control')

    assert len(controls) == 1
    assert controls[0].attrib['name'] == 'k1_mujoco'
    assert [
        plugin.text for plugin in root.findall('.//plugin')
    ] == ['ai_sapiens_mujoco/MujocoSystem']
    assert root.find(".//param[@name='viewer']").text.lower() == 'false'
    assert root.find(".//param[@name='gantry']").text.lower() == 'true'
    assert root.find(".//param[@name='scene_file']").text.endswith(
        '/mujoco/k1/scene_gantry.xml'
    )


def test_xacro_expands_with_mujoco_and_radiomaster_usb():
    """Ensure USB RC replaces the MuJoCo-provided hat interfaces."""
    document = xacro.process_file(
        str(XACRO_PATH),
        mappings={
            'sim_mujoco': 'true',
            'mujoco_viewer': 'false',
            'radiomaster_usb': 'true',
            'radiomaster_usb_device': '/dev/input/js7',
        },
    )
    root = ET.fromstring(document.toxml())
    controls = root.findall('ros2_control')

    assert [control.attrib['name'] for control in controls] == [
        'k1_mujoco',
        'k1_radiomaster_usb',
    ]
    assert [plugin.text for plugin in root.findall('.//plugin')] == [
        'ai_sapiens_mujoco/MujocoSystem',
        (
            'radiomaster_usb_hardware_interface/'
            'RadiomasterUsbHardwareInterface'
        ),
    ]
    assert controls[0].find("./sensor[@name='hat']") is None
    assert controls[1].find("./sensor[@name='hat']") is not None
    assert controls[1].find(".//param[@name='device']").text == '/dev/input/js7'


def test_xacro_ignores_radiomaster_usb_without_mujoco():
    """Ensure real hardware always retains the K1 HAT system."""
    document = xacro.process_file(
        str(XACRO_PATH),
        mappings={
            'use_mock_hardware': 'true',
            'mock_sensor_commands': 'true',
            'radiomaster_usb': 'true',
            'radiomaster_usb_device': '/dev/input/js7',
        },
    )
    root = ET.fromstring(document.toxml())
    controls = root.findall('ros2_control')

    assert len(controls) == 6
    assert 'k1_hat' in {control.attrib['name'] for control in controls}
    assert 'k1_radiomaster_usb' not in {
        control.attrib['name'] for control in controls
    }
    assert {
        plugin.text for plugin in root.findall('.//plugin')
    } == {'mock_components/GenericSystem'}


def test_mujoco_model_matches_urdf():
    """Keep MuJoCo kinematics, inertias, limits, and actuators synchronized."""
    _, urdf_links, urdf_joints = _urdf_model()
    mjcf = ET.parse(MJCF_PATH).getroot()
    bodies = {body.attrib['name']: body for body in mjcf.findall('.//body')}
    mjcf_joints = {
        joint.attrib['name']: joint
        for joint in mjcf.findall('.//joint')
        if 'name' in joint.attrib
    }
    motors = {motor.attrib['joint']: motor for motor in mjcf.findall('.//actuator/motor')}
    movable = {
        name: joint
        for name, joint in urdf_joints.items()
        if joint.attrib['type'] != 'fixed'
    }

    assert set(bodies) == set(urdf_links)
    assert set(mjcf_joints) == set(movable)
    assert set(motors) == set(movable)
    assert _values(bodies['pelvis'].attrib['pos']) == (0.0, 0.0, 0.7955)

    for name, urdf_link in urdf_links.items():
        urdf_inertial = urdf_link.find('inertial')
        mjcf_inertial = bodies[name].find('inertial')
        assert math.isclose(
            float(mjcf_inertial.attrib['mass']),
            float(urdf_inertial.find('mass').attrib['value']),
        )

        values = urdf_inertial.find('inertia').attrib
        expected = tuple(
            float(values[key])
            for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')
        )
        assert _values(mjcf_inertial.attrib['fullinertia']) == expected

    for name, urdf_joint in movable.items():
        mjcf_joint = mjcf_joints[name]
        limit = urdf_joint.find('limit').attrib
        expected_range = (float(limit['lower']), float(limit['upper']))
        assert _values(mjcf_joint.attrib['range']) == expected_range
        assert _values(mjcf_joint.attrib['axis']) == _values(
            urdf_joint.find('axis').attrib['xyz']
        )

        child = urdf_joint.find('child').attrib['link']
        expected_position = _values(urdf_joint.find('origin').attrib['xyz'])
        actual_position = _values(bodies[child].attrib.get('pos', '0 0 0'))
        assert actual_position == expected_position

        expected_class = 'qc060' if _is_qc060_joint(name) else 'qc080'
        assert motors[name].attrib['class'] == expected_class
        assert mjcf_joint.attrib['class'] == _armature_class(name)

    defaults = {
        default.attrib['class']: default
        for default in mjcf.findall('./default/default')
    }
    for class_name, expected_armature in ARMATURE_BY_CLASS.items():
        actual_armature = float(defaults[class_name].find('joint').attrib['armature'])
        assert math.isclose(actual_armature, expected_armature)

    assert _values(defaults['qc060'].find('motor').attrib['ctrlrange']) == (
        -47.277,
        47.277,
    )
    assert _values(defaults['qc080'].find('motor').attrib['ctrlrange']) == (
        -96.864,
        96.864,
    )
    speed_limits = {
        numeric.attrib['name']: float(numeric.attrib['data'])
        for numeric in mjcf.findall('./custom/numeric')
    }
    assert math.isclose(speed_limits['qc060_max_speed_rad_s'], 20.943951)
    assert math.isclose(speed_limits['qc080_max_speed_rad_s'], 11.519173)


def test_mujoco_scene_has_floor():
    """Ensure the dynamic scene includes the K1 model and a contact floor."""
    scene = ET.parse(SCENE_PATH).getroot()
    assert scene.find('include').attrib['file'] == 'k1.xml'
    floor = scene.find(".//geom[@name='floor']")
    assert floor is not None
    assert floor.attrib['type'] == 'plane'


@pytest.mark.skipif(find_spec('mujoco') is None, reason='MuJoCo Python is not installed')
def test_mujoco_files_load():
    """Compile the standalone model and floor scene when MuJoCo is available."""
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    scene = mujoco.MjModel.from_xml_path(str(SCENE_PATH))

    assert model.nu == 23
    assert model.nq == 30
    assert model.nv == 29
    assert scene.ngeom == model.ngeom + 1

    mjcf = ET.parse(MJCF_PATH).getroot()
    for joint in mjcf.findall('.//joint'):
        if 'name' not in joint.attrib:
            continue
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint.attrib['name'])
        dof_id = model.jnt_dofadr[joint_id]
        expected_armature = ARMATURE_BY_CLASS[_armature_class(joint.attrib['name'])]
        assert math.isclose(model.dof_armature[dof_id], expected_armature)
