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
Exercise the real ONNX node with synthetic sensors and isolated command output.

Run after sourcing the workspace. No controller manager or hardware is launched.
Uses a separate localhost ROS domain and publishes commands only to a test topic.
"""

import argparse
import csv
import hashlib
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import yaml


def verify_steering_path(observations, frames, robot_heading, retargeted=False):
    """Independent scalar reconstruction of the training integration and anchor rotation."""
    def multiply(a, b):
        x, y, z, w = a
        u, v, t, s = b
        return (w*u + x*s + y*t - z*v, w*v - x*t + y*s + z*u,
                w*t + x*v - y*u + z*s, w*s - x*u - y*v - z*t)

    def yaw_quat(yaw):
        return (0.0, 0.0, math.sin(yaw/2), math.cos(yaw/2))

    yaw = 0.0
    offset = [0.0, 0.0]
    previous_frame = -1
    checked = 0
    steered_checked = 0
    for obs in observations:
        if len(obs) != 131:
            continue
        # V2 adapts legs; waist and upper body retain the source clip clock.
        indices = range(12, 23) if retargeted else range(23)
        index = min(range(len(frames)), key=lambda i: sum(
            (obs[j] - frames[i][7+j])**2 for j in indices))
        if index == previous_frame:
            continue  # debug publish may repeat a policy frame
        if index != previous_frame + 1:
            # Initial CSV rows repeat joint poses. With zero applied commands,
            # skipping those indistinguishable frames contributes no steering at all.
            assert yaw == 0.0 and not any(offset) and not any(obs[128:131]), (
                index, previous_frame)
        if index:
            vx, vy, wz = obs[128:131]
            delta_yaw = wz * 0.02
            dx, dy = [frames[index][j] - frames[index-1][j] for j in range(2)]
            c, s = math.cos(yaw + delta_yaw/2), math.sin(yaw + delta_yaw/2)
            offset[0] += c*dx - s*dy - dx + 0.02 * (
                math.cos(robot_heading)*vx - math.sin(robot_heading)*vy)
            offset[1] += s*dx + c*dy - dy + 0.02 * (
                math.sin(robot_heading)*vx + math.cos(robot_heading)*vy)
            yaw += delta_yaw
        expected = [frames[index][j] - frames[0][j] + offset[j] for j in range(2)]
        # This checks integration only. CSV contact can differ from training labels;
        # dance_reference_probe measures that difference separately.
        tolerance = .0251 if retargeted else 2e-5
        assert math.dist(obs[126:128], expected) < tolerance
        # Waist is index 12 in controller/CSV order. Synthetic measured joints stay at row zero.
        real = yaw_quat(robot_heading + frames[0][7+12])
        reference = multiply(multiply(yaw_quat(yaw), frames[index][3:7]),
                             yaw_quat(frames[index][7+12]))
        x, y, z, w = multiply((-real[0], -real[1], -real[2], real[3]), reference)
        norm = math.sqrt(x*x+y*y+z*z+w*w)
        x, y, z, w = [v/norm for v in (x, y, z, w)]
        anchor = [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*y+w*z),
                  1-2*(x*x+z*z), 2*(x*z-w*y), 2*(y*z+w*x)]
        assert max(abs(a-b) for a, b in zip(obs[46:52], anchor)) < 2e-5
        previous_frame = index
        checked += 1
        steered_checked += int(any(obs[128:131]))
    assert checked > 20, checked
    assert steered_checked > 10, steered_checked


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=187)
    policy = parser.add_mutually_exclusive_group()
    policy.add_argument(
        '--controller', action='store_true', help='Test selector 204 steering policy')
    policy.add_argument(
        '--moe', action='store_true', help='Test the configured controller MoE policy')
    policy.add_argument(
        '--redred', action='store_true', help='Test the configured Redred global XY policy')
    parser.add_argument('--cross-policy', action='store_true',
                        help='Also alternate Redred/MoE; requires --redred --full-config and keyboard')
    parser.add_argument('--teleop', choices=('keyboard', 'dualsense'), default='keyboard')
    parser.add_argument('--asset-dir', type=Path,
                        help='Test a local policy asset before installing it')
    parser.add_argument('--full-config', action='store_true',
                        help='Load every configured policy from the installed root config')
    parser.add_argument('--expected-policy-sha256',
                        help='Reject a deployed ONNX that differs from the intended export')
    args = parser.parse_args()
    if args.cross_policy and not (args.redred and args.full_config and args.teleop == 'keyboard'):
        parser.error('--cross-policy requires --redred --full-config --teleop keyboard')
    has_steering = args.controller or args.moe
    if args.redred:
        asset, mimic_state, selector = 'gloporedred', 'MimicGlopoRedred', 204
    elif args.moe:
        asset, mimic_state, selector = (
            'glopodanamite_controller_moe', 'MimicGlopodanamiteControllerMoe', 205)
    elif args.controller:
        asset, mimic_state, selector = (
            'glopodanamite_controller', 'MimicGlopodanamiteController', 204)
    else:
        asset, mimic_state, selector = 'glopodanamite', 'MimicGlopodanamite', 203
    obs_size = 131 if has_steering else 128
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from ai_sapiens_interfaces.msg import JointImpedanceCommand, KeyboardInput, ModeStatus
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState, Joy
    from std_msgs.msg import Float64MultiArray

    share = Path(get_package_share_directory('ai_sapiens_sim2real'))
    root = share / 'config/k1_config.yaml'
    config = yaml.safe_load(root.read_text())
    selectors = config['selectors']['mimic_selector']['table']
    selector = next(int(code) for code, state in selectors.items() if state == mimic_state)
    behavior = config['state_machine']['states'][mimic_state]['run']
    motion_name = config['state_behaviors'][behavior]['motion']
    bundle = args.asset_dir or share / f'assets/k1/mimic/{asset}'
    bundle = bundle.resolve()
    motion = bundle / 'params' / motion_name
    hashes = {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in (
        ('policy.onnx', bundle/'exported/policy.onnx'),
        ('sim2real.yaml', bundle/'params/sim2real.yaml'),
        (motion_name, motion))}
    if args.expected_policy_sha256 and hashes['policy.onnx'] != args.expected_policy_sha256:
        raise ValueError('Deployed policy.onnx SHA256 differs from the requested training export: '
                         f'{hashes["policy.onnx"]} != {args.expected_policy_sha256}')
    print(f'Tested assets: {bundle}', flush=True)
    for name, digest in hashes.items():
        print(f'SHA256 {name}: {digest}', flush=True)
    policy_config = yaml.safe_load((bundle/'params/sim2real.yaml').read_text())
    features = policy_config.get('commands', {}).get(
        'reference_trajectory', {}).get('required_runtime_features', [])
    retargeted = any(feature in features for feature in (
        'dance_motion_reference_v2', 'dance_motion_reference_v3', 'dance_motion_reference_v4'))
    with motion.open() as source:
        frames = [[float(x) for x in row] for _, row in zip(range(500), csv.reader(source))]
    first = frames[0]
    origin = policy_config.get('commands', {}).get(
        'reference_trajectory', {}).get('observation_origin', 'episode')
    assert origin in ('episode', 'motion'), origin
    entry_xy = first[:2] if origin == 'motion' else [0.0, 0.0]
    reference_origin = [0.0, 0.0] if origin == 'motion' else first[:2]
    velocities = []
    for index in range(len(frames)):
        left, right = max(index - 1, 0), min(index + 1, len(frames) - 1)
        velocities.append([(frames[right][7 + j] - frames[left][7 + j]) /
                           ((right - left) * 0.02) for j in range(23)])
    binary = (Path(get_package_prefix('ai_sapiens_sim2real')) /
              'lib/ai_sapiens_sim2real/ai_sapiens_sim2real_node')

    with tempfile.TemporaryDirectory(prefix='gloposition-smoke-') as directory:
        temporary = Path(directory)
        # Exercise the selected policy and fallback without needing every local asset.
        config['policy_asset_roots'] = [str(share/'assets/k1')]
        config['teleop_input']['config'] = str(share/'config'/config['teleop_input']['config'])
        if not args.full_config:
            config['state_machine']['states'] = {
                name: value for name, value in config['state_machine']['states'].items()
                if name in ('Damping', 'ReadyPose', 'Velocity', 'Mimic', mimic_state)}
            config['selectors']['mimic_selector']['table'] = {selector: mimic_state}
            config['state_behaviors'] = {
                name: value for name, value in config['state_behaviors'].items()
                if name in ('damping', 'ready_pose', 'velocity_policy', behavior)}
        if args.asset_dir:
            config['state_behaviors'][behavior] = {
                'kind': 'mimic', 'policy_path': str(bundle/'exported/policy.onnx'),
                'sim2real_yaml_path': str(bundle/'params/sim2real.yaml'),
                'motion_file': str(motion)}
        root = temporary/'k1_config.yaml'
        root.write_text(yaml.safe_dump(config))
        os.environ['ROS_LOG_DIR'] = str(temporary / 'ros_logs')
        keyboard = yaml.safe_load((share / f'config/teleop/{args.teleop}.yaml').read_text())
        if args.teleop == 'dualsense':
            keyboard['selector_navigation']['initial_code'] = selector
        keyboard['topic'] = '/test_gloposition/keyboard'
        keyboard_path = temporary / 'keyboard.yaml'
        keyboard_path.write_text(yaml.safe_dump(keyboard))
        parameters = {
            'config_path': str(root),
            'teleop_input_plugin': ('ai_sapiens_sim2real/DualSenseTeleopInputPlugin' if
                                    args.teleop == 'dualsense' else
                                    'ai_sapiens_sim2real/KeyboardTeleopInputPlugin'),
            'teleop_input_config_path': str(keyboard_path),
            'imu_topic': '/test_gloposition/imu',
            'joint_states_topic': '/test_gloposition/joints',
            'localization_topic': '/test_gloposition/odom',
            'joint_command_topic': '/test_gloposition/commands',
            'mode_status_topic': '/test_gloposition/mode',
            'control_rate': '50.0',  # publish every policy frame, including entry
            'lock_memory': 'false',
            'thread_priority': '0',
            'localization_timeout': '0.15',
            'debug_publish_enabled': 'true',
            'wait_for_ready_timeout': '15.0',
        }
        command = [str(binary), '--ros-args']
        for key, value in parameters.items():
            command += ['-p', f'{key}:={value}']

        rclpy.init()
        node = rclpy.create_node('gloposition_smoke_test')
        pubs = {
            'imu': node.create_publisher(Imu, '/test_gloposition/imu', 10),
            'joints': node.create_publisher(JointState, '/test_gloposition/joints', 10),
            'keyboard': node.create_publisher(
                Joy if args.teleop == 'dualsense' else KeyboardInput, keyboard['topic'], 10),
            'odom': node.create_publisher(Odometry, '/test_gloposition/odom', 10),
        }
        state = {'mode': None, 'command': None, 'obs': None, 'obs_count': 0}
        observations = []

        def observe(msg):
            state['obs'] = list(msg.data)
            state['obs_count'] += 1
            observations.append(state['obs'])

        def receive_command(msg):
            for name in ('positions', 'feedforward', 'kp', 'kd'):
                values = getattr(msg, name)
                assert len(values) == len(config['robot_joint_order']), name
                assert all(math.isfinite(value) for value in values), name
            state['command'] = msg

        subscriptions = [
            node.create_subscription(ModeStatus, '/test_gloposition/mode',
                                     lambda m: state.update(mode=m.active_mode), 10),
            node.create_subscription(JointImpedanceCommand, '/test_gloposition/commands',
                                     receive_command, 10),
            node.create_subscription(
                Float64MultiArray, '/policy_input/raw_observation', observe, 10),
        ]
        sequence = 0
        input_code = 1
        velocity = [0.0, 0.0, 0.0]
        send_odom = False
        freeze_odom_stamp = False
        last_odom_stamp = None
        odom_xy = [1.0, 2.0]
        odom_yaw = 0.3
        odom_roll = 0.0
        imu_yaw = 0.0
        imu_orientation_available = True

        log = (temporary / 'runtime.log').open('w+')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

        def drive(seconds, condition=None):
            nonlocal sequence, last_odom_stamp
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise AssertionError(f'Runtime exited with {process.returncode}')
                stamp = node.get_clock().now().to_msg()
                imu = Imu()
                imu.header.stamp = stamp
                imu.orientation.w = math.cos(imu_yaw / 2)
                imu.orientation.z = math.sin(imu_yaw / 2)
                imu.orientation_covariance[0] = 0.0 if imu_orientation_available else -1.0
                imu.linear_acceleration.z = 9.81
                pubs['imu'].publish(imu)
                joints = JointState()
                joints.header.stamp = stamp
                joints.name = config['robot_joint_order']
                joints.position = first[7:]
                joints.velocity = [0.0] * 23
                pubs['joints'].publish(joints)
                key = KeyboardInput()
                key.header.stamp = stamp
                sequence += 1
                key.sequence = sequence
                key.input_code = input_code
                key.selector_code = selector
                key.linear_x, key.linear_y, key.angular_z = velocity
                if args.teleop == 'dualsense':
                    joy = Joy()
                    joy.header.stamp = stamp
                    joy.axes = [0.0] * 8
                    joy.buttons = [0] * 15
                    if input_code:
                        joy.buttons[{1: 1, 2: 0, 3: 3, 4: 2}[input_code]] = 1
                    deadzone = keyboard['deadzone']
                    for axis, value in zip((1, 0, 2), velocity):
                        magnitude = deadzone + (1-deadzone) * abs(value)
                        joy.axes[axis] = math.copysign(magnitude, value) if value else 0.0
                    pubs['keyboard'].publish(joy)
                else:
                    pubs['keyboard'].publish(key)
                if send_odom:
                    odom = Odometry()
                    if not freeze_odom_stamp:
                        last_odom_stamp = stamp
                    odom.header.stamp = last_odom_stamp
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'pelvis'
                    odom.pose.pose.position.x, odom.pose.pose.position.y = odom_xy
                    odom.pose.pose.orientation.w = math.cos(odom_yaw / 2) * math.cos(odom_roll / 2)
                    odom.pose.pose.orientation.x = math.cos(odom_yaw / 2) * math.sin(odom_roll / 2)
                    odom.pose.pose.orientation.y = math.sin(odom_yaw / 2) * math.sin(odom_roll / 2)
                    odom.pose.pose.orientation.z = math.sin(odom_yaw / 2) * math.cos(odom_roll / 2)
                    pubs['odom'].publish(odom)
                    pubs['odom'].publish(odom)  # estimator contact-event duplicate
                rclpy.spin_once(node, timeout_sec=0.005)
                if condition is not None and condition():
                    return
                time.sleep(0.005)
            if condition is not None:
                raise AssertionError(f'Timed out: mode={state["mode"]}')

        def assert_entry_coordinates(begin):
            entry = next((obs for obs in observations[begin:] if len(obs) == obs_size and
                          max(abs(obs[j] - first[7 + j]) for j in range(23)) < 1e-5), None)
            assert entry is not None, 'Did not observe the first motion frame'
            expected = entry_xy * 2 + ([0.0] * 3 if has_steering else [])
            assert max(abs(a - b) for a, b in zip(entry[124:obs_size], expected)) < 1e-6, (
                entry[124:obs_size], expected)
            return entry

        try:
            drive(12, lambda: state['mode'] == 'Damping')
            input_code = 2
            drive(3, lambda: state['mode'] == 'ReadyPose')
            input_code = 4
            drive(3, lambda: state['mode'] == 'Velocity')
            drive(0.1)
            assert state['command'] is not None
            assert state['mode'] == 'Velocity', 'Held mimic request retriggered entry'
            assert any(k > 0 for k in state['command'].kp)
            print('PASS: missing localization rejects mimic entry and runs Velocity')

            send_odom = True
            input_code = 2
            drive(3, lambda: state['mode'] == 'ReadyPose')
            before = state['obs_count']
            entry_begin = before
            input_code = 4
            drive(3, lambda: state['mode'] == mimic_state and state['obs_count'] > before)
            input_code = 0
            drive(0.05)
            obs = state['obs']
            assert len(obs) == obs_size, len(obs)
            assert max(abs(obs[124 + i] - entry_xy[i]) for i in range(2)) < 1e-6
            assert all(math.isfinite(x) for x in obs)
            first_entry = assert_entry_coordinates(before)
            print(f'PASS: real ONNX uses {origin} origin; entry XY={entry_xy}')
            # Repeated startup poses can have different boundary derivatives.
            # Match both halves of motion_command to identify a source frame.
            frame_index = min(range(len(frames)), key=lambda i: sum(
                (obs[j] - frames[i][7 + j]) ** 2 +
                (obs[23 + j] - velocities[i][j]) ** 2 for j in range(23)))
            assert max(abs(obs[j] - frames[frame_index][7 + j]) for j in range(23)) < 1e-5
            expected_velocity = velocities[frame_index]
            assert max(abs(obs[23 + j] - expected_velocity[j]) for j in range(23)) < 1e-4
            assert max(abs(obs[126 + j] - (frames[frame_index][j] - reference_origin[j]))
                       for j in range(2)) < 1e-6
            print('PASS: zero-command motion positions and velocities match the source CSV')

            odom_xy[0] += 0.1
            before = state['obs_count']
            drive(0.07)
            assert state['obs_count'] > before
            norm = math.sqrt(sum(v*v for v in first[3:7]))
            qx, qy, qz, qw = [v/norm for v in first[3:7]]
            ref_yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))
            delta = ref_yaw - odom_yaw
            expected = [entry_xy[0] + 0.1 * math.cos(delta),
                        entry_xy[1] + 0.1 * math.sin(delta)]
            assert max(abs(state['obs'][124 + i] - expected[i]) for i in range(2)) < 1e-4
            print('PASS: estimator displacement updates XY in a fixed motion frame')

            drive(0.2)
            assert state['mode'] == mimic_state
            print('PASS: duplicate odometry timestamps keep mimic running')

            if has_steering:
                velocity = [0.5, -0.4, 0.6]
                drive(0.4)
                applied = state['obs'][128:131]
                print(f'{args.teleop}: normalized={velocity}, applied obs={applied}')
                target = [0.15, -0.12, 0.18]
                assert all(0 < v / t < 1 for v, t in zip(applied, target)), applied
                # Constant commands follow the exact continuous-time first-order filter.
                alpha = -math.expm1(-0.02 / 0.5)
                unique = []
                for o in observations:
                    if len(o) == obs_size and (not unique or o[:23] != unique[-1][:23]):
                        unique.append(o)
                last = unique[-3:]
                for previous, current in zip(last, last[1:]):
                    expected = [v + alpha * (t - v) for v, t in zip(previous[128:131], target)]
                    assert max(abs(a - b) for a, b in zip(current[128:131], expected)) < 1e-6, (
                        previous[128:131], current[128:131], expected)
                verify_steering_path(observations[entry_begin:], frames, ref_yaw, retargeted)
                if retargeted:
                    index = min(range(len(frames)), key=lambda i: sum(
                        (state['obs'][j] - frames[i][7+j])**2 for j in range(12, 23)))
                    assert max(abs(state['obs'][j] - frames[index][7+j])
                               for j in range(12)) > 1e-4
                    print('PASS: steering supplies adapted leg references to the real MoE ONNX')
                # Normalized 0.3 maps to 0.09 m/s or rad/s: inside the 0.1 deadband.
                velocity = [0.3, -0.3, 0.3]
                drive(0.2)
                assert all(0 < v / p < 1 for v, p in zip(state['obs'][128:131], applied))

                def assert_reached_xy():
                    obs = state['obs']
                    assert math.dist(obs[124:126], obs[126:128]) < (
                        .0251 if retargeted else 1e-5)

                assert_reached_xy()
                odom_xy[0] += 0.03  # The robot still moves during deceleration.
                drive(0.08)
                assert_reached_xy()
                drive(5, lambda: all(v == 0.0 for v in state['obs'][128:131]) and
                      math.dist(state['obs'][124:126], state['obs'][126:128]) < (
                          .0251 if retargeted else 1e-5))
                assert_reached_xy()
                settled = state['obs'][:]
                drive(0.12)

                def frame_index_for(obs):
                    return min(range(len(frames)), key=lambda i: sum(
                        (obs[j] - frames[i][7+j])**2
                        for j in (range(12, 23) if retargeted else range(23))))

                start = frame_index_for(settled)
                end = frame_index_for(state['obs'])
                x, y, z, w = frames[start][3:7]
                norm = math.sqrt(x*x + y*y + z*z + w*w)
                x, y, z, w = [v/norm for v in (x, y, z, w)]
                waist = frames[start][19]
                heading = math.atan2(
                    2*(x*y+w*z)*math.cos(waist) + (1-2*(x*x+z*z))*math.sin(waist),
                    (1-2*(y*y+z*z))*math.cos(waist) + 2*(x*y-w*z)*math.sin(waist))
                # Infer the completed steering yaw from the anchor observation.
                yaw_offset = (ref_yaw + first[19] +
                              math.atan2(settled[48], settled[46]) - heading)
                dx, dy = [frames[end][j] - frames[start][j] for j in range(2)]
                expected = [settled[126] + math.cos(yaw_offset)*dx - math.sin(yaw_offset)*dy,
                            settled[127] + math.sin(yaw_offset)*dx + math.cos(yaw_offset)*dy]
                assert math.dist(state['obs'][126:128], expected) < (
                    .0501 if retargeted else 2e-5)
                # Settled dancing must no longer follow arbitrary odometry motion.
                old_robot_xy = state['obs'][124:126]
                odom_xy[1] += 0.4
                drive(0.08)
                assert math.dist(state['obs'][124:126], old_robot_xy) > 0.39
                assert math.dist(state['obs'][124:126], state['obs'][126:128]) > 0.3
                assert state['mode'] == mimic_state
                print('PASS: commands below 0.1 release missed motion and resume the clip')
                print('PASS: 131D applied commands, smoothing/release, steered XY and anchor yaw')
                # Re-entry with a held command must restart applied velocity from zero.
                velocity = [0.5, -0.4, 0.6]

            # Keep the same estimator stream while walking and turning between dances.
            input_code = 3
            drive(3, lambda: state['mode'] == 'Velocity')
            odom_xy[0] += 3.0
            odom_xy[1] -= 1.5
            odom_yaw += 1.2
            if args.redred:
                odom_roll = 0.45  # Residual estimator tilt from the previous dance.
                imu_yaw = -0.7  # IMU heading need not share the odometry origin.
            drive(0.15)
            before = state['obs_count']
            input_code = 4
            drive(3, lambda: state['mode'] == mimic_state and state['obs_count'] > before)
            input_code = 0
            drive(0.06)
            reentry = assert_entry_coordinates(before)
            if args.redred:
                assert max(abs(a-b) for a,b in zip(first_entry[46:52], reentry[46:52])) < 1e-5
                assert max(abs(v) for v in reentry[101:124]) < 1e-6
                print('PASS: reentry rejects estimator tilt, realigns IMU yaw, resets last_action')
            assert max(abs(state['obs'][124 + i] - entry_xy[i]) for i in range(2)) < 1e-6
            odom_xy[1] += 0.1
            drive(0.08)
            delta = ref_yaw - odom_yaw
            expected = [entry_xy[0] - 0.1 * math.sin(delta),
                        entry_xy[1] + 0.1 * math.cos(delta)]
            assert max(abs(state['obs'][124 + i] - expected[i]) for i in range(2)) < 1e-4
            print('PASS: Velocity -> Mimic after translation/turn captures new XY and yaw offsets')

            if has_steering:
                before = state['obs_count']
                odom_xy[0] += 2.0
                drive(0.12)
                assert state['mode'] == mimic_state
                assert state['obs_count'] > before
                assert math.dist(state['obs'][124:126], state['obs'][126:128]) > 1.5
                odom_xy[0] -= 2.0
                drive(0.06)
                print('PASS: root tracking error above 1.5 m keeps Mimic running')

            if args.redred:
                imu_orientation_available = False
                drive(3, lambda: state['mode'] == 'Velocity')
                imu_orientation_available = True
                drive(0.1)
                input_code = 4
                drive(3, lambda: state['mode'] == mimic_state)
                input_code = 0
                print('PASS: unavailable IMU attitude blocks policy and recovers after valid reentry')

            freeze_odom_stamp = True
            drive(3, lambda: state['mode'] == 'Velocity')
            drive(0.05)
            assert any(k > 0 for k in state['command'].kp)
            print('PASS: repeated frozen timestamps time out into Velocity')

            freeze_odom_stamp = False
            input_code = 2
            drive(3, lambda: state['mode'] == 'ReadyPose')
            input_code = 4
            drive(3, lambda: state['mode'] == mimic_state)
            input_code = 0
            send_odom = False
            drive(3, lambda: state['mode'] == 'Velocity')
            drive(0.05)
            assert any(k > 0 for k in state['command'].kp)
            print('PASS: missing odometry stops mimic and runs Velocity')

            if args.cross_policy:
                # One process and one estimator stream for all transitions.
                # Hold the measured joints fixed to separate policy reset/input
                # semantics from physical initial-pose and balance differences.
                moe_state = 'MimicGlopodanamiteControllerMoe'
                moe_selector = next(int(code) for code, name in selectors.items() if name == moe_state)
                redred_selector = next(int(c) for c, n in selectors.items() if n == mimic_state)
                moe_behavior = config['state_machine']['states'][moe_state]['run']
                moe_motion = share / 'assets/k1/mimic/glopodanamite_controller_moe/params' / config[
                    'state_behaviors'][moe_behavior]['motion']
                with moe_motion.open() as source:
                    moe_first = [float(x) for x in next(csv.reader(source))]

                def enter_policy(name, code, size, row):
                    nonlocal input_code, selector
                    input_code = 3
                    drive(3, lambda: state['mode'] == 'Velocity')
                    input_code = 0
                    selector = code
                    drive(.08)
                    begin = len(observations)
                    input_code = 4
                    drive(3, lambda: state['mode'] == name and len(observations) > begin)
                    input_code = 0
                    drive(.08)
                    entry = next((obs for obs in observations[begin:] if len(obs) == size and
                                  max(abs(obs[j]-row[7+j]) for j in range(23)) < 1e-5), None)
                    assert entry is not None, f'Missing first frame for {name}'
                    assert max(abs(v) for v in entry[101:124]) < 1e-6, 'Leaked previous action'
                    return entry

                send_odom = True
                odom_roll = 0.0
                imu_yaw = 0.0
                velocity = [.5, -.4, .6]  # MoE must still start with zero applied steering.
                baseline = enter_policy(moe_state, moe_selector, 131, moe_first)
                assert max(abs(v) for v in baseline[124:131]) < 1e-6
                for cycle in range(3):
                    odom_xy[0] += 2.0
                    odom_xy[1] -= 1.0
                    odom_yaw += .6
                    odom_roll = .15 * (cycle + 1)
                    imu_yaw -= .4
                    redred = enter_policy(mimic_state, redred_selector, 128, first)
                    assert max(abs(a-b) for a, b in zip(redred[46:52], first_entry[46:52])) < 1e-5
                    returned = enter_policy(moe_state, moe_selector, 131, moe_first)
                    error = max(abs(a-b) for a, b in zip(baseline, returned))
                    print(f'Cross-policy cycle {cycle+1}: max first-observation delta={error:.8f}',
                          flush=True)
                    assert error < 1e-5, 'MoE input changed after Redred despite identical robot pose'
                    assert max(abs(v) for v in returned[124:131]) < 1e-6
                print('PASS: three Redred/MoE cycles reset origin, attitude, time, action and steering')
        except Exception:
            log.flush()
            print((temporary / 'runtime.log').read_text()[-16000:])
            raise
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            log.close()
            del subscriptions
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
