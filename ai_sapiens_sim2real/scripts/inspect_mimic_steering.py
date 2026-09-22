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

"""Read teleop input and policy tracking error without issuing robot commands."""

import math
import time

from ai_sapiens_interfaces.msg import KeyboardInput, ModeStatus, RcStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


def main():
    rclpy.init()
    node = rclpy.create_node('inspect_mimic_steering')
    samples = {}
    peak_xy_error = 0.0

    def receive(name, message):
        nonlocal peak_xy_error
        if name == 'obs' and len(message.data) == 131:
            peak_xy_error = max(peak_xy_error,
                                math.dist(message.data[124:126], message.data[126:128]))
        samples[name] = (message, time.monotonic())

    subscriptions = [
        node.create_subscription(ModeStatus, '/ai_sapiens/mode_status',
                                 lambda m: receive('mode', m), 10),
        node.create_subscription(Joy, '/joy', lambda m: receive('joy', m),
                                 qos_profile_sensor_data),
        node.create_subscription(KeyboardInput, '/keyboard_teleop/input',
                                 lambda m: receive('keyboard', m), 10),
        node.create_subscription(RcStatus, '/ai_sapiens_rc/status',
                                 lambda m: receive('rc', m), 10),
        node.create_subscription(Float64MultiArray, '/policy_input/raw_observation',
                                 lambda m: receive('obs', m), 10),
    ]

    def report():
        nonlocal peak_xy_error
        now = time.monotonic()
        active = 'unknown'
        if 'mode' in samples:
            mode, stamp = samples['mode']
            active = mode.active_mode
            print(f'mode={active} authority={mode.authority} age={now-stamp:.1f}s')
        else:
            print('mode: no data')
        if 'joy' in samples:
            joy, stamp = samples['joy']
            axes = [joy.axes[i] if i < len(joy.axes) else None for i in (1, 0, 2)]
            print(f'joy raw [vx,vy,yaw]={axes} age={now-stamp:.1f}s')
        else:
            print('joy: no data')
        if 'keyboard' in samples:
            key, stamp = samples['keyboard']
            print(f'keyboard latched [vx,vy,yaw]={key.linear_x, key.linear_y, key.angular_z}'
                  f' age={now-stamp:.1f}s (Space clears the command)')
        if 'rc' in samples:
            rc, stamp = samples['rc']
            axes = {c.rc_channel: round(c.axis, 6) for c in rc.channels
                    if c.rc_channel in (1, 3, 4)}
            print(f'RC raw axes={axes} age={now-stamp:.1f}s')
        if 'obs' not in samples:
            print('obs: no data; launch policy with debug_publish_enabled:=true', flush=True)
            return
        obs, stamp = samples['obs']
        values = obs.data
        print(f'obs_size={len(values)} age={now-stamp:.1f}s')
        steering_modes = ('MimicGlopodanamiteController', 'MimicGlopodanamiteControllerMoe')
        if active in steering_modes and len(values) == 131:
            print('applied [vx,vy,yaw]=' + str([round(x, 5) for x in values[128:131]]))
            print('robot_xy=' + str([round(x, 5) for x in values[124:126]]) +
                  ' reference_xy=' + str([round(x, 5) for x in values[126:128]]))
            error = math.dist(values[124:126], values[126:128])
            print(f'XY error={error:.4f}m, peak since last report={peak_xy_error:.4f}m')
        peak_xy_error = 0.0
        print(flush=True)

    timer = node.create_timer(1.0, report)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        del timer, subscriptions
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
