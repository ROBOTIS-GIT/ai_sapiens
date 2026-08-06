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
# Author: Woojin Wie

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('start_rviz', default_value='false',
                              description='Whether to execute rviz2'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware mirroring command.'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false',
                              description='Enable mock sensor commands.'),
        DeclareLaunchArgument('model', default_value='k1_rev1',
                              description='Model name.'),
        DeclareLaunchArgument('radiomaster_pocket_usb', default_value='false',
                              description='Use RadioMaster Pocket USB instead of the K1 HAT.'),
        DeclareLaunchArgument('radiomaster_pocket_usb_device', default_value='/dev/input/js0',
                              description='Linux joystick device read by RadioMaster hardware.'),
    ]

    start_rviz = LaunchConfiguration('start_rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    model = LaunchConfiguration('model')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('ai_sapiens_description'),
                              'urdf',
                              'k1_rev1',
                              'k1.urdf.xacro']),
        ' ',
        'use_mock_hardware:=', use_mock_hardware,
        ' ',
        'mock_sensor_commands:=', mock_sensor_commands,
        ' ',
        'model:=', model,
        ' ',
        'radiomaster_pocket_usb:=', LaunchConfiguration('radiomaster_pocket_usb'),
        ' ',
        'radiomaster_pocket_usb_device:=',
        LaunchConfiguration('radiomaster_pocket_usb_device'),
    ])

    controller_manager_config = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_bringup'), 'config',
        'k1_rev1', 'k1_rev1_controllers.yaml'
    ])
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_description'), 'rviz', 'k1.rviz'
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_config],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(start_rviz)
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'imu_sensor_broadcaster',
        ],
        output='screen'
    )

    rc_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rc_broadcaster'],
        output='screen'
    )

    joint_group_impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_impedance_controller'],
        output='screen'
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )

    return LaunchDescription(
        declared_arguments + [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            rc_broadcaster_spawner,
            joint_group_impedance_controller_spawner,
            delay_rviz_after_joint_state_broadcaster_spawner,
            # rviz_node,
        ]
    )
