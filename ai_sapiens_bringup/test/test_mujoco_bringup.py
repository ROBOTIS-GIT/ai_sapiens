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

import unittest

from ai_sapiens_interfaces.msg import RcStatus
from ai_sapiens_interfaces.srv import SetGantryHeight
from controller_manager_msgs.srv import ListControllers
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import launch_testing.actions
import launch_testing.markers
from launch_ros.substitutions import FindPackageShare
import pytest
import rclpy
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ai_sapiens_bringup'), 'launch', 'k1.launch.py'])),
        launch_arguments={
            'sim_mujoco': 'true',
            'mujoco_viewer': 'false',
            'mujoco_gantry': 'true',
        }.items())
    return launch.LaunchDescription([bringup, launch_testing.actions.ReadyToTest()])


class TestMujocoBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('mujoco_bringup_checker')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_msg(self, msg_type, topic, timeout=30.0):
        received = []
        sub = self.node.create_subscription(msg_type, topic, received.append, 10)
        try:
            end = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)
            while not received and self.node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(self.node, timeout_sec=0.2)
            self.assertTrue(received, f'no message on {topic}')
            return received[0]
        finally:
            self.node.destroy_subscription(sub)

    def _call(self, client, request, timeout=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout))
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        self.assertIsNotNone(fut.result())
        return fut.result()

    def test_1_controllers_active(self):
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        expected = {
            'joint_state_broadcaster', 'imu_sensor_broadcaster',
            'rc_broadcaster', 'joint_group_impedance_controller'}
        deadline = self.node.get_clock().now().nanoseconds + int(60 * 1e9)
        active = set()
        while active != expected and self.node.get_clock().now().nanoseconds < deadline:
            result = self._call(client, ListControllers.Request(), timeout=30.0)
            active = {c.name for c in result.controller if c.state == 'active'} & expected
            if active != expected:
                rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertEqual(active, expected)

    def test_2_topics_publish(self):
        joint_state = self._wait_msg(JointState, '/joint_states')
        self.assertEqual(len(joint_state.name), 23)
        self._wait_msg(Imu, '/imu_sensor_broadcaster/imu')
        status = self._wait_msg(RcStatus, '/ai_sapiens_rc/status')
        self.assertEqual(status.crsf_failsafe, 0)

    def test_3_gantry_sequence(self):
        set_h = self.node.create_client(
            SetGantryHeight, '/mujoco_sim/gantry/set_height')
        release = self.node.create_client(Trigger, '/mujoco_sim/gantry/release')
        res = self._call(set_h, SetGantryHeight.Request(height=1.45, speed=0.2))
        self.assertTrue(res.success)
        rel = self._call(release, Trigger.Request())
        self.assertTrue(rel.success)
        res2 = self._call(set_h, SetGantryHeight.Request(height=1.6, speed=0.2))
        self.assertFalse(res2.success)
