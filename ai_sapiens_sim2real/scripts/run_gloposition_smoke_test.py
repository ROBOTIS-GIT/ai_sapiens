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
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import yaml


def verify_steering_path(observations, frames, robot_heading):
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
        index = min(range(len(frames)), key=lambda i: sum(
            (obs[j] - frames[i][7+j])**2 for j in range(23)))
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
        assert max(abs(a-b) for a, b in zip(obs[126:128], expected)) < 2e-5
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
    parser.add_argument(
        '--controller', action='store_true', help='Test selector 204 steering policy')
    parser.add_argument('--teleop', choices=('keyboard', 'dualsense'), default='keyboard')
    args = parser.parse_args()
    asset = 'glopodanamite_controller' if args.controller else 'glopodanamite'
    mimic_state = 'MimicGlopodanamiteController' if args.controller else 'MimicGlopodanamite'
    obs_size = 131 if args.controller else 128
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
    motion = share / f'assets/k1/mimic/{asset}/params/dynamite004_headwrap_v3.csv'
    with motion.open() as source:
        frames = [[float(x) for x in row] for _, row in zip(range(200), csv.reader(source))]
    first = frames[0]
    binary = (Path(get_package_prefix('ai_sapiens_sim2real')) /
              'lib/ai_sapiens_sim2real/ai_sapiens_sim2real_node')

    with tempfile.TemporaryDirectory(prefix='gloposition-smoke-') as directory:
        temporary = Path(directory)
        os.environ['ROS_LOG_DIR'] = str(temporary / 'ros_logs')
        keyboard = yaml.safe_load((share / f'config/teleop/{args.teleop}.yaml').read_text())
        if args.teleop == 'dualsense':
            keyboard['selector_navigation']['initial_code'] = 204 if args.controller else 203
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
        subscriptions = [
            node.create_subscription(ModeStatus, '/test_gloposition/mode',
                                     lambda m: state.update(mode=m.active_mode), 10),
            node.create_subscription(JointImpedanceCommand, '/test_gloposition/commands',
                                     lambda m: state.update(command=m), 10),
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
                imu.orientation.w = 1.0
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
                key.selector_code = 204 if args.controller else 203
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
                    odom.pose.pose.orientation.w = math.cos(odom_yaw / 2)
                    odom.pose.pose.orientation.z = math.sin(odom_yaw / 2)
                    pubs['odom'].publish(odom)
                    pubs['odom'].publish(odom)  # estimator contact-event duplicate
                rclpy.spin_once(node, timeout_sec=0.005)
                if condition is not None and condition():
                    return
                time.sleep(0.005)
            if condition is not None:
                raise AssertionError(f'Timed out: mode={state["mode"]}')

        def assert_zero_entry(begin):
            entry = next((obs for obs in observations[begin:] if len(obs) == obs_size and
                          max(abs(obs[j] - first[7 + j]) for j in range(23)) < 1e-5), None)
            assert entry is not None, 'Did not observe the first motion frame'
            assert max(abs(x) for x in entry[124:obs_size]) < 1e-6, entry[124:obs_size]

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
            assert max(abs(obs[124 + i]) for i in range(2)) < 1e-6
            assert all(math.isfinite(x) for x in obs)
            assert_zero_entry(before)
            print('PASS: real ONNX enters with robot and reference XY both zero')
            frame_index = min(range(len(frames)), key=lambda i: sum(
                (obs[j] - frames[i][7 + j]) ** 2 for j in range(23)))
            assert max(abs(obs[j] - frames[frame_index][7 + j]) for j in range(23)) < 1e-5
            left, right = max(frame_index - 1, 0), min(frame_index + 1, len(frames) - 1)
            expected_velocity = [
                (frames[right][7 + j] - frames[left][7 + j]) / ((right - left) * 0.02)
                for j in range(23)]
            assert max(abs(obs[23 + j] - expected_velocity[j]) for j in range(23)) < 1e-4
            assert max(abs(obs[126 + j] - (frames[frame_index][j] - first[j]))
                       for j in range(2)) < 1e-6
            print('PASS: omitted motion_format selects MJLab frames and central velocities')

            odom_xy[0] += 0.1
            before = state['obs_count']
            drive(0.07)
            assert state['obs_count'] > before
            ref_yaw = math.atan2(2 * first[6] * first[5], 1 - 2 * first[5] ** 2)
            delta = ref_yaw - odom_yaw
            expected = [0.1 * math.cos(delta), 0.1 * math.sin(delta)]
            assert max(abs(state['obs'][124 + i] - expected[i]) for i in range(2)) < 1e-4
            print('PASS: estimator displacement updates XY in a fixed motion frame')

            drive(0.2)
            assert state['mode'] == mimic_state
            print('PASS: duplicate odometry timestamps keep mimic running')

            if args.controller:
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
                velocity = [0.0, 0.0, 0.0]
                drive(0.2)
                assert all(0 < v / p < 1 for v, p in zip(state['obs'][128:131], applied))
                verify_steering_path(observations[entry_begin:], frames, ref_yaw)
                print('PASS: 131D applied commands, smoothing/release, steered XY and anchor yaw')
                # Re-entry with a held command must restart applied velocity from zero.
                velocity = [0.5, -0.4, 0.6]

            # Keep the same estimator stream while walking and turning between dances.
            input_code = 3
            drive(3, lambda: state['mode'] == 'Velocity')
            odom_xy[0] += 3.0
            odom_xy[1] -= 1.5
            odom_yaw += 1.2
            drive(0.15)
            before = state['obs_count']
            input_code = 4
            drive(3, lambda: state['mode'] == mimic_state and state['obs_count'] > before)
            input_code = 0
            drive(0.06)
            assert_zero_entry(before)
            assert max(abs(state['obs'][124 + i]) for i in range(2)) < 1e-6
            odom_xy[1] += 0.1
            drive(0.08)
            delta = ref_yaw - odom_yaw
            expected = [-0.1 * math.sin(delta), 0.1 * math.cos(delta)]
            assert max(abs(state['obs'][124 + i] - expected[i]) for i in range(2)) < 1e-4
            print('PASS: Velocity -> Mimic after translation/turn captures new XY and yaw offsets')

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
