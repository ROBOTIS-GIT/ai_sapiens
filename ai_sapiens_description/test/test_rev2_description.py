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

"""Protect the rev2 geometry/physics contract without adding linkage DOFs."""

from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]


def values(text):
    """Parse an XML attribute containing space-separated numbers."""
    return tuple(map(float, text.split()))


class Rev2DescriptionTests(unittest.TestCase):
    """Validate the revision's assets, frames, inertials and runtime topology."""

    def setUp(self):
        """Read the independent standalone descriptions for comparison."""
        self.old = ET.parse(PACKAGE / 'urdf/k1_rev1/k1.urdf').getroot()
        self.urdf = ET.parse(PACKAGE / 'urdf/k1_rev2/k1.urdf').getroot()
        self.mjcf = ET.parse(PACKAGE / 'mujoco/k1_rev2/k1.xml').getroot()

    def test_training_joint_contract(self):
        """Preserve controlled joints without adding a passive linkage loop."""
        old = {j.get('name'): j for j in self.old.findall('joint')}
        new = {j.get('name'): j for j in self.urdf.findall('joint')}
        self.assertEqual(set(old), set(new))
        self.assertEqual(len(self.urdf.findall('link')), 25)
        self.assertEqual(sum(j.get('type') == 'revolute' for j in new.values()), 23)
        self.assertFalse(self.urdf.findall('.//mimic'))
        self.assertFalse(self.mjcf.findall('equality/*'))
        for name, j in new.items():
            for tag in ['parent', 'child', 'axis', 'limit', 'dynamics']:
                a, b = old[name].find(tag), j.find(tag)
                self.assertEqual(a.attrib if a is not None else None,
                                 b.attrib if b is not None else None)

    def test_reviewed_coordinate_changes(self):
        """Check the reviewed PDF offsets and unchanged lower-body frames."""
        joints = {j.get('name'): j for j in self.urdf.findall('joint')}
        expected = {
            'right_shoulder_pitch_joint': (0.0015, -0.068, 0.319),
            'right_shoulder_roll_joint': (-0.006, -0.079, 0),
            'right_shoulder_yaw_joint': (0.001, 0, -0.0677),
            'right_elbow_joint': (0.016, 0.008, -0.1173),
            'right_wrist_roll_joint': (0.0677, -0.001, 0),
            'left_elbow_joint': (0.016000144, -0.007999684, -0.117300003),
            'left_wrist_roll_joint': (0.06769998, 0.001001335, -0.000000012),
        }
        for name, xyz in expected.items():
            self.assertEqual(values(joints[name].find('origin').get('xyz')), xyz)
        for j in self.old.findall('joint'):
            if any(key in j.get('name') for key in ['hip_', 'knee_', 'ankle_']):
                self.assertEqual(values(j.find('origin').get('xyz')),
                                 values(joints[j.get('name')].find('origin').get('xyz')))

    def test_asset_paths_and_materials(self):
        """Resolve visual and collision assets from the package directory."""
        for link in self.urdf.findall('link'):
            self.assertEqual(len(link.findall('visual')), 1)
            self.assertIsNone(link.find('visual/material'))
            self.assertTrue(link.find('visual/geometry/mesh').get('filename').endswith('.dae'))
        for mesh in self.urdf.findall('.//mesh'):
            filename = mesh.get('filename')
            package_prefix = 'package://ai_sapiens_description/'
            if filename.startswith(package_prefix):
                path = PACKAGE / filename.removeprefix(package_prefix)
            else:
                path = PACKAGE / 'urdf/k1_rev2' / filename
            self.assertTrue(path.is_file(), filename)

    def test_mass_frames_and_mjcf_match(self):
        """Keep URDF and MJCF inertials in the same center-of-mass frames."""
        bodies = {b.get('name'): b for b in self.mjcf.findall('.//body')}
        joints = {j.find('child').get('link'): j for j in self.urdf.findall('joint')}
        self.assertEqual(set(bodies), {link.get('name') for link in self.urdf.findall('link')})
        total = 0
        for link in self.urdf.findall('link'):
            name = link.get('name')
            i = link.find('inertial')
            b = bodies[name]
            total += float(i.find('mass').get('value'))
            self.assertEqual(float(i.find('mass').get('value')),
                             float(b.find('inertial').get('mass')))
            self.assertEqual(values(i.find('origin').get('xyz')),
                             values(b.find('inertial').get('pos')))
            keys = ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz']
            tensor = [float(i.find('inertia').get(k)) for k in keys]
            self.assertEqual(tuple(tensor), values(b.find('inertial').get('fullinertia')))
            a, d, f, c, e, g = tensor
            self.assertGreater(a * d * f + 2 * c * e * g - a * g * g - d * e * e - f * c * c, 0)
            if name != 'pelvis':
                self.assertEqual(values(joints[name].find('origin').get('xyz')),
                                 values(b.get('pos')))
        self.assertAlmostEqual(total, 37.55070742, places=8)

    def test_collision_coverage(self):
        """Match collision meshes and keep shoulder and ankle pitch links collision-free."""
        for link in self.urdf.findall('link'):
            name = link.get('name')
            with self.subTest(link=name):
                collisions = link.findall('collision')
                geoms = self.mjcf.findall(
                    f".//body[@name='{name}']/geom[@class='collision']")
                if name.endswith(('_ankle_pitch_link', '_shoulder_pitch_link')):
                    self.assertFalse(collisions)
                    self.assertFalse(geoms)
                    continue
                urdf_meshes = [
                    shape.get('filename').split('/meshes/k1_rev2/', 1)[1]
                    for shape in link.findall('collision/geometry/mesh')
                ]
                mjcf_meshes = [
                    self.mjcf.find(f"asset/mesh[@name='{geom.get('mesh')}']").get('file')
                    for geom in geoms if geom.get('type') == 'mesh'
                ]
                self.assertCountEqual(urdf_meshes, mjcf_meshes)
                if name.endswith('_wrist_roll_rubber_hand'):
                    side = name.split('_', 1)[0]
                    self.assertCountEqual(urdf_meshes, [
                        f'collision/{name}.stl',
                        f'collision/{side}_wrist_roll_forearm.stl',
                    ])
                    self.assertEqual(len(collisions), 2)
                    self.assertEqual(len(geoms), 2)
                    continue
                self.assertEqual(len(urdf_meshes), 1)
                if name == 'torso_link':
                    self.assertEqual(len(collisions), 2)
                    self.assertEqual(len(geoms), 2)
                    neck = link.find("collision[@name='torso_neck_collision']")
                    box, = [g for g in geoms if g.get('type') == 'box']
                    self.assertEqual(box.get('name'), neck.get('name'))
                    self.assertEqual(values(box.get('pos')),
                                     values(neck.find('origin').get('xyz')))
                    self.assertEqual(tuple(2 * x for x in values(box.get('size'))),
                                     values(neck.find('geometry/box').get('size')))
                else:
                    self.assertEqual(len(collisions), 1)
                    self.assertEqual(len(geoms), 1)

    def test_runtime_xacro_variants(self):
        """Expand mock, MuJoCo and RC variants with local package resolution."""
        import xacro
        import xacro.substitution_args

        cases = [
            ({'use_mock_hardware': 'true'}, 6, None),
            ({'sim_mujoco': 'true', 'mujoco_gantry': 'false'}, 1, 'scene.xml'),
            ({'sim_mujoco': 'true', 'mujoco_gantry': 'true'}, 1, 'scene_gantry.xml'),
            ({'sim_mujoco': 'true', 'radiomaster_usb': 'true'}, 2, 'scene_gantry.xml'),
        ]
        expected_joints = {j.get('name') for j in self.urdf.findall('joint')}
        for options, control_count, scene in cases:
            with self.subTest(options=options):
                # Isolate Xacro file generation from the host ROS installation.
                with patch.object(xacro.substitution_args, '_eval_find') as find_package:
                    find_package.side_effect = lambda name: str(PACKAGE.parent / name)
                    document = xacro.process_file(
                        str(PACKAGE / 'urdf/k1_rev2/k1.urdf.xacro'), mappings=options)
                root = ET.fromstring(document.toxml())
                self.assertEqual(len(root.findall('ros2_control')), control_count)
                self.assertEqual({j.get('name') for j in root.findall('joint')}, expected_joints)
                for link in root.findall('link'):
                    source = self.urdf.find(f"link[@name='{link.get('name')}']")
                    for tag in ('mass', 'origin', 'inertia'):
                        self.assertEqual(link.find(f'inertial/{tag}').attrib,
                                         source.find(f'inertial/{tag}').attrib)
                    actual = [Path(mesh.get('filename')).name
                              for mesh in link.findall('collision/geometry/mesh')]
                    expected = [Path(mesh.get('filename')).name
                                for mesh in source.findall('collision/geometry/mesh')]
                    self.assertEqual(actual, expected)
                    if link.get('name') == 'torso_link':
                        neck_xpath = "collision[@name='torso_neck_collision']"
                        for tag in ('origin', 'geometry/box'):
                            self.assertEqual(link.find(f'{neck_xpath}/{tag}').attrib,
                                             source.find(f'{neck_xpath}/{tag}').attrib)
                if scene:
                    expected = PACKAGE / 'mujoco/k1_rev2' / scene
                    self.assertEqual(root.find(".//param[@name='scene_file']").text,
                                     str(expected))
                    self.assertTrue(expected.is_file())
                    self.assertEqual(len(root.find('ros2_control').findall('joint')), 23)

    def test_mujoco_training_state_and_steps(self):
        """Load every scene and verify compiled inertials and finite dynamics."""
        try:
            import mujoco
            import numpy as np
        except ImportError:
            self.skipTest('MuJoCo is not installed')
        scenes = [('k1.xml', 0, 26), ('scene.xml', 0, 26), ('scene_gantry.xml', 1, 27)]
        for scene, neq, nbody in scenes:
            m = mujoco.MjModel.from_xml_path(str(PACKAGE / 'mujoco/k1_rev2' / scene))
            self.assertEqual((m.nq, m.nv, m.nu, m.neq, m.nbody), (30, 29, 23, neq, nbody))
            for link in self.urdf.findall('link'):
                body = m.body(link.get('name')).id
                inertial = link.find('inertial')
                self.assertEqual(m.body_mass[body], float(inertial.find('mass').get('value')))
                np.testing.assert_allclose(
                    m.body_ipos[body], values(inertial.find('origin').get('xyz')),
                    rtol=0, atol=1e-12)
                v = inertial.find('inertia').attrib
                tensor = np.array([
                    [float(v['ixx']), float(v['ixy']), float(v['ixz'])],
                    [float(v['ixy']), float(v['iyy']), float(v['iyz'])],
                    [float(v['ixz']), float(v['iyz']), float(v['izz'])],
                ])
                rotation = np.zeros((3, 3))
                mujoco.mju_quat2Mat(rotation.ravel(), m.body_iquat[body])
                compiled = rotation @ np.diag(m.body_inertia[body]) @ rotation.T
                # Test the full tensor, allowing one ppm for engine diagonalization.
                # A standalone fullinertia body reproduces the same small residual.
                relative_error = np.linalg.norm(compiled - tensor) / np.linalg.norm(tensor)
                self.assertLess(relative_error, 1e-6, link.get('name'))
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            for contact in d.contact:
                if contact.dist < 0:
                    bodies = m.geom_bodyid[[contact.geom1, contact.geom2]]
                    self.assertIn(0, bodies, 'Neutral pose has a self-collision')
            if scene == 'scene.xml':
                d.qpos[2] -= 0.001
                mujoco.mj_forward(m, d)
                floor = m.geom('floor').id
                touching = {index for contact in d.contact if contact.dist <= 0
                            for index in (contact.geom1, contact.geom2)
                            if floor in (contact.geom1, contact.geom2) and index != floor}
                feet = {m.geom(f'{side}_ankle_roll_link_collision_0').id
                        for side in ('left', 'right')}
                self.assertEqual(touching, feet)
                mujoco.mj_resetData(m, d)
            for _ in range(1000):
                mujoco.mj_step(m, d)
            self.assertTrue(np.isfinite(d.qpos).all())
            self.assertTrue(np.isfinite(d.qvel).all())
            self.assertFalse(any(w.number for w in d.warning))


if __name__ == '__main__':
    unittest.main()
