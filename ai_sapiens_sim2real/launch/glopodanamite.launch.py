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
# Author: Woojin Wie, Kiwoong Park

"""
Run the localization estimator and the existing K1 policy node.

Start k1_mujoco.launch.py or k1.launch.py separately for robot feedback.
The estimator is activated with its existing enable_inekf_orientation service.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('teleop_input_plugin', default_value=''),
        DeclareLaunchArgument('teleop_input_config_path', default_value=''),
        DeclareLaunchArgument('command_publisher_enabled', default_value='true'),
        DeclareLaunchArgument('localization_align_on_entry', default_value='true'),
        DeclareLaunchArgument('localization_timeout', default_value='0.5'),
        DeclareLaunchArgument('debug_publish_enabled', default_value='false'),
        DeclareLaunchArgument('estimator_type', default_value='invariant_graph'),
    ]
    estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ai_sapiens_state_estimator'), 'launch',
            'state_estimator.launch.py'])),
        launch_arguments={
            'launch_rviz': 'false',
            'estimator_type': LaunchConfiguration('estimator_type'),
        }.items(),
    )
    policy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ai_sapiens_sim2real'), 'launch',
            'ai_sapiens_sim2real.launch.py'])),
        launch_arguments={
            'robot': 'k1',
            'localization_topic': '/state_estimator/odom',
            'localization_world_frame': 'odom',
            'localization_base_frame': 'pelvis',
            **{name: LaunchConfiguration(name) for name in (
                'teleop_input_plugin', 'teleop_input_config_path',
                'command_publisher_enabled', 'localization_align_on_entry',
                'localization_timeout', 'debug_publish_enabled')},
        }.items(),
    )
    return LaunchDescription(arguments + [estimator, policy])
