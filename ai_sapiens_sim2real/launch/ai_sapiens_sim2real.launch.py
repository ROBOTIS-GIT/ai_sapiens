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
Launch file for ai_sapiens_sim2real controller node.

This node runs ONNX policy inference with realtime-safe architecture.
It subscribes to IMU and joint states topics, runs inference at the
configured rate, and outputs joint position commands.

Usage:
    ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py robot:=k1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('ai_sapiens_sim2real'),
        'config',
        PythonExpression(["'", LaunchConfiguration('robot'), "_config.yaml'"]),
    ])

    # ==========================================================================
    # Launch Arguments
    # ==========================================================================
    args = [
        DeclareLaunchArgument(
            'robot',
            default_value='k1',
            description='Robot config name; resolves to config/<robot>_config.yaml by default'
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu_sensor_broadcaster/imu',
            description='IMU sensor topic'
        ),
        DeclareLaunchArgument(
            'joint_states_topic',
            default_value='/joint_states',
            description='Joint states topic'
        ),
        DeclareLaunchArgument(
            'thread_priority',
            default_value='50',
            description='RT thread priority (0-99)'
        ),
        DeclareLaunchArgument(
            'lock_memory',
            default_value='true',
            description='Lock memory to prevent page faults'
        ),
        DeclareLaunchArgument(
            'wait_for_ready_timeout',
            default_value='30.0',
            description='Timeout (seconds) to wait for sensors (0 = no timeout)'
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='1000.0',
            description='Command publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'status_log_enabled',
            default_value='false',
            description='Enable periodic RT-loop status logs'
        ),
        DeclareLaunchArgument(
            'detailed_status_log',
            default_value='false',
            description='Print detailed IMU, joint, and action samples in status logs'
        ),
        DeclareLaunchArgument(
            'debug_publish_enabled',
            default_value='false',
            description='Enable RT-loop observation and raw debug publishers'
        ),
        DeclareLaunchArgument(
            'joint_command_topic',
            default_value='/joint_group_impedance_controller/commands',
            description='Joint command topic'
        ),
        DeclareLaunchArgument(
            'command_publisher_enabled',
            default_value='true',
            description='Enable publishing joint commands'
        ),
        DeclareLaunchArgument(
            'enforce_position_limits',
            default_value='false',
            description='Clamp published commands to active policy position limits'
        ),
        DeclareLaunchArgument(
            'api_heartbeat_topic',
            default_value='/ai_sapiens/api_heartbeat',
            description='API heartbeat topic'
        ),
        DeclareLaunchArgument(
            'api_heartbeat_timeout',
            default_value='0.2',
            description='API heartbeat sequence watchdog timeout in seconds'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='API mode velocity command topic'
        ),
        DeclareLaunchArgument(
            'set_mode_by_name_service',
            default_value='/ai_sapiens/set_mode_by_name',
            description='Set mode service using FSM mode name'
        ),
    ]

    # ==========================================================================
    # Node
    # ==========================================================================
    ai_sapiens_sim2real_node = Node(
        package='ai_sapiens_sim2real',
        executable='ai_sapiens_sim2real_node',
        name='ai_sapiens_sim2real_node',
        output='screen',
        parameters=[{
            'config_path': default_config,
            'imu_topic': LaunchConfiguration('imu_topic'),
            'joint_states_topic': LaunchConfiguration('joint_states_topic'),
            'thread_priority': LaunchConfiguration('thread_priority'),
            'lock_memory': LaunchConfiguration('lock_memory'),
            'wait_for_ready_timeout': LaunchConfiguration('wait_for_ready_timeout'),
            'control_rate': LaunchConfiguration('control_rate'),
            'status_log_enabled': LaunchConfiguration('status_log_enabled'),
            'detailed_status_log': LaunchConfiguration('detailed_status_log'),
            'debug_publish_enabled': LaunchConfiguration('debug_publish_enabled'),
            'joint_command_topic': LaunchConfiguration('joint_command_topic'),
            'command_publisher_enabled': LaunchConfiguration('command_publisher_enabled'),
            'enforce_position_limits': LaunchConfiguration('enforce_position_limits'),
            'api_heartbeat_topic': LaunchConfiguration('api_heartbeat_topic'),
            'api_heartbeat_timeout': LaunchConfiguration('api_heartbeat_timeout'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'set_mode_by_name_service': LaunchConfiguration('set_mode_by_name_service'),
        }],
    )

    return LaunchDescription(args + [ai_sapiens_sim2real_node])
