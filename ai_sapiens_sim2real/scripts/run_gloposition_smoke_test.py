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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=187)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from ai_sapiens_interfaces.msg import JointImpedanceCommand, KeyboardInput, ModeStatus
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray

    share = Path(get_package_share_directory('ai_sapiens_sim2real'))
    root = share / 'config/k1_config.yaml'
    config = yaml.safe_load(root.read_text())
    motion = share / 'assets/k1/mimic/glopodanamite/params/dynamite004_headwrap_v3.csv'
    with motion.open() as source:
        frames = [[float(x) for x in row] for _, row in zip(range(50), csv.reader(source))]
    first = frames[0]
    binary = (Path(get_package_prefix('ai_sapiens_sim2real')) /
              'lib/ai_sapiens_sim2real/ai_sapiens_sim2real_node')

    with tempfile.TemporaryDirectory(prefix='gloposition-smoke-') as directory:
        temporary = Path(directory)
        os.environ['ROS_LOG_DIR'] = str(temporary / 'ros_logs')
        keyboard = yaml.safe_load((share / 'config/teleop/keyboard.yaml').read_text())
        keyboard['topic'] = '/test_gloposition/keyboard'
        keyboard_path = temporary / 'keyboard.yaml'
        keyboard_path.write_text(yaml.safe_dump(keyboard))
        parameters = {
            'config_path': str(root),
            'teleop_input_plugin': 'ai_sapiens_sim2real/KeyboardTeleopInputPlugin',
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
            'keyboard': node.create_publisher(KeyboardInput, keyboard['topic'], 10),
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
                key.selector_code = 203
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
            entry = next((obs for obs in observations[begin:] if len(obs) == 128 and
                          max(abs(obs[j] - first[7 + j]) for j in range(23)) < 1e-5), None)
            assert entry is not None, 'Did not observe the first motion frame'
            assert max(abs(x) for x in entry[124:128]) < 1e-6, entry[124:128]

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
            input_code = 4
            drive(3, lambda: state['mode'] == 'MimicGlopodanamite' and state['obs_count'] > before)
            input_code = 0
            drive(0.05)
            obs = state['obs']
            assert len(obs) == 128, len(obs)
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
            assert state['mode'] == 'MimicGlopodanamite'
            print('PASS: duplicate odometry timestamps keep mimic running')

            # Keep the same estimator stream while walking and turning between dances.
            input_code = 3
            drive(3, lambda: state['mode'] == 'Velocity')
            odom_xy[0] += 3.0
            odom_xy[1] -= 1.5
            odom_yaw += 1.2
            drive(0.15)
            before = state['obs_count']
            input_code = 4
            drive(3, lambda: state['mode'] == 'MimicGlopodanamite' and state['obs_count'] > before)
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
            drive(3, lambda: state['mode'] == 'MimicGlopodanamite')
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
