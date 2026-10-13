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

"""Run BTS Boy With Luv with robot IMU attitude in simulation or on hardware.

This orientation-only model has no XY input and does not require an estimator.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('start_simulator', default_value='false',
                              description='Start MuJoCo if no robot/simulator is running'),
        DeclareLaunchArgument('mujoco_viewer', default_value='true'),
        DeclareLaunchArgument('mujoco_gantry', default_value='true'),
        DeclareLaunchArgument('radiomaster_usb', default_value='false'),
        DeclareLaunchArgument('teleop_input_plugin', default_value=''),
        DeclareLaunchArgument('teleop_input_config_path', default_value=''),
        DeclareLaunchArgument('command_publisher_enabled', default_value='true'),
        DeclareLaunchArgument('debug_publish_enabled', default_value='false'),
    ]
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ai_sapiens_bringup'), 'launch', 'k1_mujoco.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_simulator')),
        launch_arguments={name: LaunchConfiguration(name) for name in (
            'mujoco_viewer', 'mujoco_gantry', 'radiomaster_usb')}.items(),
    )
    policy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ai_sapiens_sim2real'), 'launch',
            'ai_sapiens_sim2real.launch.py'])),
        launch_arguments={
            'robot': 'k1_bts_boywithluv_a2t',
            **{name: LaunchConfiguration(name) for name in (
                'teleop_input_plugin', 'teleop_input_config_path',
                'command_publisher_enabled', 'debug_publish_enabled')},
        }.items(),
    )
    return LaunchDescription(arguments + [simulator, policy])
