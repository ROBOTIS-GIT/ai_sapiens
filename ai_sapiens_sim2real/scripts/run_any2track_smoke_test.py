#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software distributed
# under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied.
"""Exercise native ROS Any2Track entry, histories, odometry loss/reentry and completion.

Synthetic sensors and isolated ROS domain; no physical robot or balance test.
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
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--domain-id', type=int, default=189)
    p.add_argument('--robot', default='k1_any2track', help='Root config name without _config.yaml')
    args = p.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from ai_sapiens_interfaces.msg import JointImpedanceCommand, KeyboardInput, ModeStatus
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Float64MultiArray

    share = Path(get_package_share_directory('ai_sapiens_sim2real'))
    config = yaml.safe_load((share / f'config/{args.robot}_config.yaml').read_text())
    table = config['selectors']['mimic_selector']['table']
    if len(table) != 1:
        raise ValueError('Smoke test requires a dedicated config with one mimic selector')
    selector, mode_name = next(iter(table.items()))
    behavior_name = config['state_machine']['states'][mode_name]['run']
    behavior = config['state_behaviors'][behavior_name]
    bundle = share / 'assets/k1' / behavior['asset']
    deployment = yaml.safe_load((bundle / 'params/sim2real.yaml').read_text())
    path_tracking = deployment['runtime_type'] == 'any2track_gt_xy_adapter'
    obs_size = 134 if path_tracking else 132
    with (bundle / 'params' / behavior['motion']).open() as stream:
        rows = [list(map(float, row)) for row in csv.reader(stream) if row]
    # ROS test sensors come from the original CSV, not generated deployment files.
    frames = []
    for i in range(2):
        row = rows[i]
        lo, hi = max(0, i - 1), i + 1
        dt = (hi - lo) * deployment['step_dt']
        qpos = row[:3] + [row[6]] + row[3:6] + row[7:]
        qvel = [(rows[hi][j] - rows[lo][j]) / dt for j in range(3)] + [0.0] * 3
        qvel += [(rows[hi][j] - rows[lo][j]) / dt for j in range(7, 30)]
        frames.append(qpos + qvel)
    frame0, frame1 = frames
    robot_xy = list(frame0[:2])
    robot_q = list(frame0[3:7])
    # Deliberately disagree with the IMU, including tilt, to detect estimator leakage.
    odom_q = [math.cos(0.4), math.sin(0.4), 0.0, 0.0]
    imu_valid = True
    config['policy_asset_roots'] = [str(share / 'assets/k1')]
    config['teleop_input']['config'] = str(share / 'config/teleop/keyboard.yaml')
    config['selectors']['mimic_selector']['table'] = {selector: mode_name}
    config['state_machine']['states'] = {
        k: v
        for k, v in config['state_machine']['states'].items()
        if k in ('Damping', 'ReadyPose', 'Velocity', 'Mimic', mode_name)
    }
    config['state_behaviors'] = {
        k: v
        for k, v in config['state_behaviors'].items()
        if k in ('damping', 'ready_pose', 'velocity_policy', behavior_name)
    }
    config['state_behaviors'][behavior_name]['time_end'] = 0.8
    with tempfile.TemporaryDirectory(prefix='any2track-smoke-') as folder:
        folder = Path(folder)
        (folder / 'root.yaml').write_text(yaml.safe_dump(config))
        keyboard = yaml.safe_load((share / 'config/teleop/keyboard.yaml').read_text())
        keyboard['topic'] = '/test_any2track/keyboard'
        (folder / 'keyboard.yaml').write_text(yaml.safe_dump(keyboard))
        os.environ['ROS_LOG_DIR'] = str(folder / 'logs')
        params = {
            'config_path': str(folder / 'root.yaml'),
            'teleop_input_plugin': 'ai_sapiens_sim2real/KeyboardTeleopInputPlugin',
            'teleop_input_config_path': str(folder / 'keyboard.yaml'),
            'imu_topic': '/test_any2track/imu',
            'joint_states_topic': '/test_any2track/joints',
            'localization_topic': '/test_any2track/odom',
            'joint_command_topic': '/test_any2track/commands',
            'mode_status_topic': '/test_any2track/mode',
            'control_rate': '50.0',
            'lock_memory': 'false',
            'thread_priority': '0',
            'localization_timeout': '0.15',
            'debug_publish_enabled': 'true',
        }
        binary = (
            Path(get_package_prefix('ai_sapiens_sim2real'))
            / 'lib/ai_sapiens_sim2real/ai_sapiens_sim2real_node'
        )
        command = [str(binary), '--ros-args']
        for k, v in params.items():
            command += ['-p', f'{k}:={v}']
        rclpy.init()
        node = rclpy.create_node('any2track_smoke_test')
        pubs = {
            k: node.create_publisher(t, f'/test_any2track/{k}', 10)
            for k, t in [
                ('imu', Imu),
                ('joints', JointState),
                ('odom', Odometry),
                ('keyboard', KeyboardInput),
            ]
        }
        state = {'mode': None, 'commands': 0}
        observations = []

        def receive_command(msg):
            for field in ('positions', 'feedforward', 'kp', 'kd'):
                values = getattr(msg, field)
                assert len(values) == 23 and all(math.isfinite(v) for v in values)
            state['commands'] += 1

        subscriptions = [
            node.create_subscription(
                ModeStatus,
                '/test_any2track/mode',
                lambda m: state.update(mode=m.active_mode),
                10,
            ),
            node.create_subscription(
                Float64MultiArray,
                '/policy_input/raw_observation',
                lambda m: observations.append(list(m.data)),
                10,
            ),
            node.create_subscription(
                JointImpedanceCommand, '/test_any2track/commands', receive_command, 10
            ),
        ]
        code, sequence, send_odom = 1, 0, False
        log = (folder / 'runtime.log').open('w+')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

        def drive(seconds, condition=None):
            nonlocal sequence
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                assert process.poll() is None, f'Runtime exited: {process.returncode}'
                stamp = node.get_clock().now().to_msg()
                imu = Imu()
                imu.header.stamp = stamp
                imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = (
                    robot_q
                )
                imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
                    frame0[33:36]
                )
                imu.orientation_covariance[0] = 0.0 if imu_valid else -1.0
                imu.linear_acceleration.z = 9.81
                pubs['imu'].publish(imu)
                joints = JointState()
                joints.header.stamp = stamp
                joints.name = config['robot_joint_order']
                joints.position = frame0[7:30]
                joints.velocity = frame0[36:59]
                pubs['joints'].publish(joints)
                key = KeyboardInput()
                key.header.stamp = stamp
                sequence += 1
                key.sequence = sequence
                key.input_code = code
                key.selector_code = selector
                pubs['keyboard'].publish(key)
                if send_odom:
                    odom = Odometry()
                    odom.header.stamp = stamp
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'pelvis'
                    odom.pose.pose.position.x, odom.pose.pose.position.y = robot_xy
                    (odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                     odom.pose.pose.orientation.y, odom.pose.pose.orientation.z) = odom_q
                    pubs['odom'].publish(odom)
                rclpy.spin_once(node, timeout_sec=0.005)
                if condition and condition():
                    return
                time.sleep(0.005)
            if condition:
                raise AssertionError(f'Timeout in mode {state["mode"]}')

        def entry(begin):
            found = [
                o
                for o in observations[begin:]
                if len(o) == obs_size
                and max(abs(o[j] - (frame1[7 + j] - frame0[7 + j])) for j in range(23)) < 1e-5
            ]
            assert found, 'Missing upcoming-frame entry observation'
            assert max(abs(found[0][98 + j] - frame0[7 + j]) for j in range(23)) < 1e-6, (
                'Previous motor target leaked'
            )
            return found[0]

        def check_imu_gravity(observation):
            w, x, y, z = robot_q
            norm2 = w * w + x * x + y * y + z * z
            expected = [2 * (w * y - x * z) / norm2,
                        -2 * (y * z + w * x) / norm2,
                        -(w * w - x * x - y * y + z * z) / norm2]
            assert max(abs(a - b) for a, b in zip(observation[46:49], expected)) < 1e-5, (
                'Projected gravity must use the robot IMU, not estimator attitude'
            )

        try:
            drive(12, lambda: state['mode'] == 'Damping')
            code = 2
            drive(3, lambda: state['mode'] == 'ReadyPose')
            code = 4
            if path_tracking:
                drive(3, lambda: state['mode'] == 'Velocity')
                print('PASS: missing odometry rejects Any2Track entry', flush=True)
            else:
                drive(3, lambda: state['mode'] == mode_name)
                print('PASS: orientation-only policy runs without odometry', flush=True)
                code = 3
                drive(3, lambda: state['mode'] == 'Velocity')
            send_odom = True
            code = 0
            drive(0.1)
            begin = len(observations)
            code = 4
            drive(3, lambda: state['mode'] == mode_name and len(observations) > begin)
            code = 0
            drive(0.06)
            first = entry(begin)
            check_imu_gravity(first)
            print(
                'PASS: three-input ONNX, upcoming reference, '
                'measured reset targets and finite commands',
                flush=True,
            )
            send_odom = False
            if path_tracking:
                drive(3, lambda: state['mode'] == 'Velocity')
                print('PASS: odometry loss hands off to Velocity', flush=True)
            else:
                drive(0.2)
                assert state['mode'] == mode_name
                print('PASS: odometry loss does not interrupt orientation-only policy', flush=True)
                code = 3
                drive(3, lambda: state['mode'] == 'Velocity')
            # Same physical pose in a different odometry world.
            angle = 0.7
            c, s = math.cos(angle / 2), math.sin(angle / 2)
            w, x, y, z = frame0[3:7]
            robot_q[:] = [c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w]
            robot_xy[:] = [6.0, -3.0]
            odom_q[:] = [math.cos(0.6), 0.0, math.sin(0.6), 0.0]
            send_odom = True
            code = 0
            drive(0.1)
            begin = len(observations)
            code = 4
            drive(3, lambda: state['mode'] == mode_name and len(observations) > begin)
            code = 0
            drive(0.06)
            second = entry(begin)
            check_imu_gravity(second)
            assert max(abs(a - b) for a, b in zip(first, second)) < 1e-5
            print('PASS: policy observations ignore estimator attitude changes', flush=True)
            drive(3, lambda: state['mode'] == 'Velocity')
            assert state['commands'] > 10
            print(
                'PASS: reentry realigns the reference, clears targets/history; '
                'finite clip completes into Velocity',
                flush=True,
            )
            imu_valid = False
            drive(0.1)
            begin = len(observations)
            code = 4
            drive(0.3)
            assert state['mode'] == 'Velocity'
            assert not any(len(o) == obs_size for o in observations[begin:])
            print('PASS: invalid IMU attitude rejects entry despite valid odometry', flush=True)
            code = 0
            imu_valid = True
            # Tilt only the IMU; the estimator pose stays fixed.
            robot_q[:] = [math.cos(0.2), math.sin(0.2), 0.0, 0.0]
            drive(0.1)
            begin = len(observations)
            code = 4
            drive(3, lambda: state['mode'] == mode_name and len(observations) > begin)
            code = 0
            drive(0.06)
            third = entry(begin)
            check_imu_gravity(third)
            assert max(abs(a - b) for a, b in zip(first[46:49], third[46:49])) > 0.1
            assert max(abs(a - b) for a, b in zip(first[121:127], third[121:127])) > 0.1
            print('PASS: IMU tilt drives gravity and relative orientation', flush=True)
        except Exception:
            log.flush()
            print((folder / 'runtime.log').read_text()[-14000:])
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
            node.destroy_node()
            rclpy.shutdown()
            del subscriptions


if __name__ == '__main__':
    main()
