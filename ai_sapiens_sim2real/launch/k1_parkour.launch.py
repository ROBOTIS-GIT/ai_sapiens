# Copyright 2026 ROBOTIS CO., LTD.
# SPDX-License-Identifier: Apache-2.0
"""Shared Parkour controller and depth pipeline; real hardware bringup is separate."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    sim = value('sim') == 'true'
    share = get_package_share_directory('ai_sapiens_sim2real')
    policy = os.path.join(share, 'assets/k1/locomotion/velocity/parkour/params/sim2real.yaml')
    actions = []
    if sim:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ai_sapiens_bringup'), 'launch/k1_mujoco.launch.py')),
            launch_arguments={'depth_policy_yaml': policy, 'parkour_physics': 'true',
                              'mujoco_viewer': value('viewer'),
                              'mujoco_gantry': value('gantry')}.items()))
    actions.append(Node(package='ai_sapiens_depth', executable='depth_history_node',
                        output='screen', parameters=[{
                            'policy_yaml': policy, 'simulated': sim,
                            'image_topic': value('depth_topic'),
                            'camera_info_topic': value('camera_info_topic'),
                            'input_frame': 'camera_link' if sim else value('depth_frame'),
                            'uint16_depth_scale': float(value('uint16_depth_scale'))}]))
    teleop = value('teleop')
    plugins = {'keyboard': 'ai_sapiens_sim2real/KeyboardTeleopInputPlugin',
               'dualsense': 'ai_sapiens_sim2real/DualSenseTeleopInputPlugin',
               'radiomaster': 'ai_sapiens_sim2real/RadiomasterPocketTeleopInputPlugin'}
    config = {'keyboard': 'keyboard', 'dualsense': 'dualsense',
              'radiomaster': 'radiomaster_pocket'}[teleop]
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        share, 'launch/ai_sapiens_sim2real.launch.py')), launch_arguments={
            'robot': 'k1', 'teleop_input_plugin': plugins[teleop],
            'teleop_input_config_path': os.path.join(share, 'config/teleop', config + '.yaml'),
            'command_publisher_enabled': value('command_publisher_enabled')}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('gantry', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='keyboard',
                              choices=['keyboard', 'dualsense', 'radiomaster']),
        DeclareLaunchArgument('command_publisher_enabled', default_value='true'),
        DeclareLaunchArgument('depth_topic', default_value='/k1/d436/depth/image_rect_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/k1/d436/depth/camera_info'),
        DeclareLaunchArgument('depth_frame', default_value='camera_depth_optical_frame'),
        DeclareLaunchArgument('uint16_depth_scale', default_value='0.001'),
        OpaqueFunction(function=setup),
    ])
