#!/usr/bin/env python3
#
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
# Author: Kiwoong Park

# Smoke tests for ai_sapiens_sim2real mode/authority transitions.
#
# Run inside the ROS Docker/container after sourcing the workspace:
#   source /opt/ros/jazzy/setup.bash
#   source /root/ros2_ws/install/setup.bash
#   python3 /root/ros2_ws/src/ai_sapiens/ai_sapiens_sim2real/scripts/run_mode_smoke_tests.py

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

LOG_DIR = Path('/tmp/ai_sapiens_mode_smoke')
ROOT_CONFIG_PATH = Path(__file__).resolve().parent.parent / 'config' / 'k1_config.yaml'
RC_CHANNEL_IDS = range(1, 17)


@dataclass(frozen=True)
class ExpectedEvent:
    active: str | None = None
    authority: str | None = None
    teleop: bool | None = None
    heartbeat: bool | None = None
    reason: str | None = None


@dataclass(frozen=True)
class Scenario:
    name: str
    duration: float
    expected_sequence: tuple[ExpectedEvent, ...]
    service_mode: str | None = None


@dataclass(frozen=True)
class RcCommand:
    input_condition: str | None
    api_mode: bool = False
    selector_state: str = 'MimicSquat'
    selector_valid: bool = True


SCENARIOS = {
    'manual_sequence': Scenario(
        name='manual_sequence',
        duration=42.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(active='Velocity', authority='MANUAL'),
            ExpectedEvent(active='MimicSquat', authority='MANUAL'),
            ExpectedEvent(active='Velocity', authority='MANUAL'),
        ),
    ),
    'manual_mimic_mid_retrigger': Scenario(
        name='manual_mimic_mid_retrigger',
        duration=44.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(active='Velocity', authority='MANUAL'),
            ExpectedEvent(active='MimicSquat', authority='MANUAL'),
            ExpectedEvent(active='Velocity', authority='MANUAL'),
            ExpectedEvent(active='MimicSquat', authority='MANUAL'),
        ),
    ),
    'manual_ready_watchdog': Scenario(
        name='manual_ready_watchdog',
        duration=16.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL', teleop=True),
            ExpectedEvent(
                active='Damping',
                authority='MANUAL',
                teleop=False,
                reason='teleop_input_timeout',
            ),
        ),
    ),
    'manual_velocity_watchdog': Scenario(
        name='manual_velocity_watchdog',
        duration=18.0,
        expected_sequence=(
            ExpectedEvent(active='Velocity', authority='MANUAL', teleop=True),
            ExpectedEvent(
                active='Damping',
                authority='MANUAL',
                teleop=False,
                reason='teleop_input_timeout',
            ),
        ),
    ),
    'manual_mimic_watchdog': Scenario(
        name='manual_mimic_watchdog',
        duration=32.0,
        expected_sequence=(
            ExpectedEvent(active='MimicSquat', authority='MANUAL', teleop=True),
            ExpectedEvent(
                active='Damping',
                authority='MANUAL',
                teleop=False,
                reason='teleop_input_timeout',
            ),
        ),
    ),
    'manual_mimic_damping_switch': Scenario(
        name='manual_mimic_damping_switch',
        duration=28.0,
        expected_sequence=(
            ExpectedEvent(active='MimicSquat', authority='MANUAL', teleop=True),
            ExpectedEvent(
                active='Damping',
                authority='MANUAL',
                teleop=True,
                reason='teleop_input_damping',
            ),
        ),
    ),
    'api_heartbeat_watchdog_ready': Scenario(
        name='api_heartbeat_watchdog_ready',
        duration=20.0,
        expected_sequence=(
            ExpectedEvent(authority='API_WARMUP', reason='api_authority_requested'),
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(
                active='ReadyPose',
                authority='MANUAL',
                heartbeat=False,
                reason='api_authority_lost',
            ),
        ),
    ),
    'api_teleop_watchdog_ready': Scenario(
        name='api_teleop_watchdog_ready',
        duration=20.0,
        expected_sequence=(
            ExpectedEvent(authority='API_WARMUP', reason='api_authority_requested'),
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(
                active='Damping',
                authority='MANUAL',
                teleop=False,
                reason='teleop_input_timeout',
            ),
        ),
    ),
    'api_mimic_heartbeat_watchdog': Scenario(
        name='api_mimic_heartbeat_watchdog',
        duration=26.0,
        service_mode='MimicSquat',
        expected_sequence=(
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(active='MimicSquat', authority='API'),
            ExpectedEvent(
                active='Velocity',
                authority='MANUAL',
                heartbeat=False,
                reason='api_authority_lost',
            ),
        ),
    ),
    'api_mimic_heartbeat_watchdog_ready_switch': Scenario(
        name='api_mimic_heartbeat_watchdog_ready_switch',
        duration=26.0,
        service_mode='MimicSquat',
        expected_sequence=(
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(active='MimicSquat', authority='API'),
            ExpectedEvent(
                active='ReadyPose',
                authority='MANUAL',
                heartbeat=False,
                reason='api_authority_lost',
            ),
        ),
    ),
    # Clean API switch release (heartbeat still valid): hand back to Manual at the
    # current switch position. Distinct from the heartbeat-loss exits above; this
    # is the only scenario exercising api_authority_released.
    'api_release_handoff': Scenario(
        name='api_release_handoff',
        duration=14.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(authority='API_WARMUP', reason='api_authority_requested'),
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(
                active='Velocity',
                authority='MANUAL',
                heartbeat=True,
                reason='api_authority_released',
            ),
        ),
    ),
    # Damping kill switch while under API: failsafe must drop to Manual + Damping
    # immediately regardless of authority (heartbeat stays valid throughout).
    'api_damping_switch': Scenario(
        name='api_damping_switch',
        duration=14.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(authority='API_WARMUP', reason='api_authority_requested'),
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(
                active='Damping',
                authority='MANUAL',
                teleop=True,
                reason='teleop_input_damping',
            ),
        ),
    ),
    # Blocked API->manual release: switch released while both active state and
    # teleop input are motion stays in API. Moving the switch to a non-motion
    # position then releases to Manual. If the block were broken, the first manual
    # event would be MimicSquat (handoff) rather than the Velocity release below, so
    # the api_authority_released-on-Velocity event would be missing.
    'api_mimic_release_blocked': Scenario(
        name='api_mimic_release_blocked',
        duration=20.0,
        service_mode='MimicSquat',
        expected_sequence=(
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(active='MimicSquat', authority='API'),
            ExpectedEvent(
                active='Velocity',
                authority='MANUAL',
                reason='api_authority_released',
            ),
        ),
    ),
    # API entry rejected for a missing heartbeat, then the consumed edge must NOT
    # re-fire when the heartbeat later appears with the switch still up; only a
    # fresh down/up toggle enters. The Velocity event proves authority stayed
    # MANUAL throughout (a manual FSM transition would be ignored under API).
    'api_entry_rejected_then_retoggle': Scenario(
        name='api_entry_rejected_then_retoggle',
        duration=22.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(
                active='Velocity', authority='MANUAL', reason='teleop_input_condition'
            ),
            ExpectedEvent(authority='API_WARMUP', reason='api_authority_requested'),
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
        ),
    ),
    # API entry attempted from a motion state (MimicSquat) is rejected; the later
    # manual MimicSquat->Velocity transition proves authority stayed MANUAL.
    'api_entry_rejected_in_motion': Scenario(
        name='api_entry_rejected_in_motion',
        duration=18.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(active='MimicSquat', authority='MANUAL'),
            ExpectedEvent(
                active='Velocity', authority='MANUAL', reason='teleop_input_condition'
            ),
        ),
    ),
    # Booting with the API switch already up (and heartbeat valid) must NOT enter
    # API: there is no rising edge from boot. The Velocity event proves it stayed
    # MANUAL; a fresh down/up toggle then enters normally.
    'api_no_entry_on_boot_switch_high': Scenario(
        name='api_no_entry_on_boot_switch_high',
        duration=22.0,
        expected_sequence=(
            ExpectedEvent(active='ReadyPose', authority='MANUAL'),
            ExpectedEvent(
                active='Velocity', authority='MANUAL', reason='teleop_input_condition'
            ),
            ExpectedEvent(authority='API_WARMUP', reason='api_authority_requested'),
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
        ),
    ),
    # Switch release coinciding with a stale heartbeat must take the heartbeat-loss
    # path, not the release path. In MimicSquat with the switch at a motion position,
    # the release path would be blocked (stuck in API); the loss path zeroes
    # velocity and substitutes Velocity for the motion handoff. Observing
    # Velocity/MANUAL/api_authority_lost proves loss took precedence over release.
    'api_release_and_loss_same_tick': Scenario(
        name='api_release_and_loss_same_tick',
        duration=18.0,
        service_mode='MimicSquat',
        expected_sequence=(
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(active='MimicSquat', authority='API'),
            ExpectedEvent(
                active='Velocity',
                authority='MANUAL',
                heartbeat=False,
                reason='api_authority_lost',
            ),
        ),
    ),
    # Release blocked by motion intent even when the selector is out of detent.
    # The Mimic switch is held while the selector is invalid, so the release must
    # stay blocked (motion intent = MimicRequested itself). Moving to a non-motion
    # switch then releases.
    'api_mimic_release_blocked_invalid_selector': Scenario(
        name='api_mimic_release_blocked_invalid_selector',
        duration=20.0,
        service_mode='MimicSquat',
        expected_sequence=(
            ExpectedEvent(authority='API', reason='api_warmup_complete'),
            ExpectedEvent(active='MimicSquat', authority='API'),
            ExpectedEvent(
                active='Velocity',
                authority='MANUAL',
                reason='api_authority_released',
            ),
        ),
    ),
}


class RcConfigEncoder:

    def __init__(self, root_config_path):
        self.root_config_path = Path(root_config_path)
        root_config = self.load_yaml(self.root_config_path)

        teleop_config_path = self.root_config_path.parent / root_config[
            'teleop_input'
        ]['config']
        self.teleop_config = self.load_yaml(teleop_config_path)
        self.topic = self.teleop_config['topic']
        self.switch_tolerance = float(
            self.teleop_config.get('switch_match_tolerance', 50.0)
        )

        self.condition_codes = {
            name: int(condition['input_code'])
            for name, condition in root_config['teleop_conditions'].items()
        }
        self.input_code_positions = self.read_input_code_positions(
            self.teleop_config['input_code']['channels']
        )
        self.api_conditions = self.read_condition_map(
            self.teleop_config.get('api_mode', {}).get('when', {})
        )
        if not self.api_conditions:
            raise ValueError(f'{teleop_config_path} must configure api_mode')

        selector_config = self.teleop_config['selector_code']
        self.selector_channel = self.parse_channel(selector_config['channel'])
        self.selector_tolerance = float(selector_config.get('tolerance', 20.0))
        self.selector_pwm_by_code = {
            int(code): float(pwm) for pwm, code in selector_config['table'].items()
        }
        self.selector_codes_by_state = self.read_selector_states(root_config)
        self.invalid_selector_pwm = self.find_invalid_selector_pwm()

        required_codes = set(self.condition_codes.values())
        missing_codes = sorted(required_codes - self.input_code_positions.keys())
        if missing_codes:
            raise ValueError(
                f'{teleop_config_path} cannot encode input codes {missing_codes}'
            )
        neutral_codes = self.input_code_positions.keys() - required_codes
        if not neutral_codes:
            raise ValueError(f'{teleop_config_path} has no neutral input-code mapping')
        self.neutral_input_code = min(neutral_codes)

    @staticmethod
    def load_yaml(path):
        with path.open(encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError(f'{path} must contain a YAML map')
        return config

    @staticmethod
    def parse_channel(channel_name):
        digits = ''.join(
            character for character in str(channel_name) if character.isdigit()
        )
        if not digits or not 1 <= int(digits) <= 16:
            raise ValueError(f'invalid RC channel: {channel_name}')
        return int(digits)

    def read_condition_map(self, condition_map):
        return [
            (self.parse_channel(channel), float(value))
            for channel, value in condition_map.items()
        ]

    def read_input_code_positions(self, channel_map):
        positions_by_code = {}

        def visit(nested_channels, parent_positions):
            for channel_name, positions in nested_channels.items():
                channel = self.parse_channel(channel_name)
                for pwm, branch in positions.items():
                    channel_positions = dict(parent_positions)
                    channel_positions[channel] = float(pwm)
                    if isinstance(branch, dict):
                        visit(branch, channel_positions)
                    else:
                        code = int(branch)
                        if code in positions_by_code:
                            raise ValueError(
                                f'input code {code} has multiple RC mappings'
                            )
                        positions_by_code[code] = channel_positions

        visit(channel_map, {})
        return positions_by_code

    @staticmethod
    def read_selector_states(root_config):
        states = {}
        for selector in root_config.get('selectors', {}).values():
            for code, state in selector.get('table', {}).items():
                if state in states:
                    raise ValueError(f'selector state {state} has multiple codes')
                states[state] = int(code)
        return states

    def find_invalid_selector_pwm(self):
        positions = sorted(self.selector_pwm_by_code.values())
        for left, right in zip(positions, positions[1:]):
            candidate = (left + right) / 2.0
            if all(
                abs(candidate - position) > self.selector_tolerance
                for position in positions
            ):
                return candidate
        raise ValueError(
            'selector_code.table has no out-of-detent value for smoke test'
        )

    def encode(self, command):
        input_code = (
            self.neutral_input_code
            if command.input_condition is None
            else self.condition_codes[command.input_condition]
        )
        assignments = dict(self.input_code_positions[input_code])

        if command.api_mode:
            for channel, pwm in self.api_conditions:
                self.assign(assignments, channel, pwm, 'API mode')
        elif self.api_conditions:
            self.disable_api_mode(assignments)

        if command.selector_valid:
            selector_code = self.selector_codes_by_state[command.selector_state]
            selector_pwm = self.selector_pwm_by_code[selector_code]
        else:
            selector_pwm = self.invalid_selector_pwm
        self.assign(assignments, self.selector_channel, selector_pwm, 'selector')

        channel_values = {channel: 1500.0 for channel in RC_CHANNEL_IDS}
        channel_values.update(assignments)
        return channel_values

    def disable_api_mode(self, assignments):
        for channel, target in self.api_conditions:
            current = assignments.get(channel)
            if current is not None and abs(current - target) > self.switch_tolerance:
                return
            if current is None:
                assignments[channel] = self.unmatched_pwm(target)
                return
        raise ValueError('input mapping cannot represent API mode off')

    def unmatched_pwm(self, target):
        candidates = (1000.0, 1500.0, 2000.0)
        candidate = max(candidates, key=lambda value: abs(value - target))
        if abs(candidate - target) <= self.switch_tolerance:
            raise ValueError(f'cannot choose an API-off value for target {target}')
        return candidate

    @staticmethod
    def assign(assignments, channel, pwm, purpose):
        existing = assignments.get(channel)
        if existing is not None and existing != pwm:
            raise ValueError(
                f'{purpose} conflicts with another mapping on RC channel {channel}'
            )
        assignments[channel] = pwm


def run_process(args, log_path):
    log = open(log_path, 'w', encoding='utf-8')
    return subprocess.Popen(
        args,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )


def terminate_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return


def kill_process(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def run_scenario(scenario):
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    scenario_log_dir = LOG_DIR / scenario.name
    scenario_log_dir.mkdir(parents=True, exist_ok=True)

    processes = []
    try:
        zenoh = run_process(
            ['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
            scenario_log_dir / 'zenoh.log',
        )
        processes.append(zenoh)
        time.sleep(2.0)

        bringup = run_process(
            [
                'ros2',
                'launch',
                'ai_sapiens_bringup',
                'k1.launch.py',
                'use_mock_hardware:=true',
            ],
            scenario_log_dir / 'bringup.log',
        )
        processes.append(bringup)
        time.sleep(8.0)

        driver_args = [
            sys.executable,
            __file__,
            '--driver',
            scenario.name,
        ]
        if scenario.service_mode:
            driver_args += ['--service-mode', scenario.service_mode]
        driver = run_process(driver_args, scenario_log_dir / 'driver.log')
        processes.append(driver)
        time.sleep(2.0)

        sim2real = run_process(
            ['ros2', 'launch', 'ai_sapiens_sim2real', 'ai_sapiens_sim2real.launch.py'],
            scenario_log_dir / 'sim2real.log',
        )
        processes.append(sim2real)

        driver_timeout = scenario.duration + 8.0
        try:
            driver.wait(timeout=driver_timeout)
        except subprocess.TimeoutExpired:
            pass

        time.sleep(1.0)
    finally:
        for process in reversed(processes):
            terminate_process(process)
        time.sleep(2.0)
        for process in reversed(processes):
            kill_process(process)

    result = parse_driver_result(scenario_log_dir / 'driver.log')
    errors = collect_errors(scenario_log_dir)
    if errors:
        result = False
    print_scenario_summary(scenario.name, result, scenario_log_dir, errors)
    return result


def parse_driver_result(driver_log_path):
    if not driver_log_path.exists():
        return False
    text = driver_log_path.read_text(encoding='utf-8', errors='replace')
    return 'RESULT PASS' in text and 'RESULT FAIL' not in text


def collect_errors(scenario_log_dir):
    errors = []
    for name in ('bringup.log', 'sim2real.log'):
        path = scenario_log_dir / name
        if not path.exists():
            continue
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
            if any(
                token in line
                for token in ('ERROR', 'FATAL', 'Exception', 'not found under')
            ):
                errors.append(f'{name}: {line}')
    return errors


def print_scenario_summary(name, passed, scenario_log_dir, errors):
    print(f'[{"PASS" if passed else "FAIL"}] {name}')
    print(f'  logs: {scenario_log_dir}')
    driver_log = scenario_log_dir / 'driver.log'
    if driver_log.exists():
        for line in driver_log.read_text(
            encoding='utf-8', errors='replace'
        ).splitlines():
            if 'EVENT ' in line or 'RESULT ' in line or 'SERVICE ' in line:
                print(f'  {line}')
    for error in errors[:20]:
        print(f'  ERROR: {error}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario',
        action='append',
        choices=sorted(SCENARIOS.keys()) + ['all'],
        default=None,
        help='Scenario to run. Can be passed multiple times. Default: all.',
    )
    parser.add_argument('--driver', choices=sorted(SCENARIOS.keys()))
    parser.add_argument('--service-mode')
    args = parser.parse_args()

    if args.driver:
        run_driver(args.driver, args.service_mode)
        return

    selected = args.scenario or ['all']
    names = list(SCENARIOS.keys()) if 'all' in selected else selected
    results = [run_scenario(SCENARIOS[name]) for name in names]
    if not all(results):
        raise SystemExit(1)


def run_driver(scenario_name, service_mode):
    import rclpy
    from ai_sapiens_interfaces.msg import RcChannel, RcStatus
    from rclpy.node import Node

    from ai_sapiens_interfaces.msg import ApiHeartbeat, ModeStatus
    from ai_sapiens_interfaces.srv import SetModeByName

    scenario = SCENARIOS[scenario_name]
    rc_config = RcConfigEncoder(ROOT_CONFIG_PATH)

    class Driver(Node):

        def __init__(self):
            super().__init__(f'{scenario_name}_driver')
            self.rc_pub = self.create_publisher(RcStatus, rc_config.topic, 10)
            self.hb_pub = self.create_publisher(
                ApiHeartbeat, '/ai_sapiens/api_heartbeat', 10
            )
            self.status_sub = self.create_subscription(
                ModeStatus, '/ai_sapiens/mode_status', self.on_status, 10
            )
            self.request_client = self.create_client(
                SetModeByName, '/ai_sapiens/set_mode_by_name'
            )
            self.start = self.get_clock().now()
            self.realtime_tick = 0
            self.heartbeat_sequence = 0
            self.events = []
            self.last_event = None
            self.service_sent = False
            self.service_done = False
            self.timer = self.create_timer(0.02, self.tick)

        def elapsed(self):
            return (self.get_clock().now() - self.start).nanoseconds / 1e9

        def tick(self):
            t = self.elapsed()
            command = rc_command_for_scenario(scenario_name, t)
            if command is not None:
                self.publish_rc(command)
            if should_publish_heartbeat(scenario_name, t):
                self.publish_heartbeat()
            if service_mode and not self.service_sent and self.has_api_authority():
                self.send_service_request(service_mode)
            if t > scenario.duration:
                self.finish()

        def publish_rc(self, command):
            values = rc_config.encode(command)
            msg = RcStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.sensor_name = 'hat'
            msg.status_topic_name = rc_config.topic
            msg.hardware_error_status = 0
            msg.realtime_tick = self.realtime_tick
            self.realtime_tick += 1
            msg.estop_active = False
            msg.crsf_last_frame_age_ms = 0
            msg.crsf_failsafe = 0
            msg.crsf_link_quality = 100
            msg.crsf_rssi_1 = 100
            msg.status_data_valid = True
            msg.realtime_tick_fresh = True
            msg.all_channels_valid = True
            msg.hardware_ok = True
            msg.estop_released = True
            msg.rc_link_ok = True
            msg.is_control_input_safe = True
            msg.channels = [
                self.make_rc_channel(channel, values[channel])
                for channel in RC_CHANNEL_IDS
            ]
            self.rc_pub.publish(msg)

        def make_rc_channel(self, channel_id, pwm):
            channel = RcChannel()
            channel.rc_channel = channel_id
            channel.name = f'RC Channel {channel_id}'
            channel.valid = True
            channel.rc_us = int(round(pwm))
            channel.axis = self.normalize_pwm(pwm)
            channel.button = 1 if pwm >= 1700.0 else 0
            channel.axis_in_deadzone = abs(pwm - 1500.0) < 50.0
            channel.button_active = channel.button != 0
            return channel

        def normalize_pwm(self, pwm):
            centered = pwm - 1500.0
            if abs(centered) < 50.0:
                return 0.0
            return max(-1.0, min(1.0, centered / 500.0))

        def publish_heartbeat(self):
            msg = ApiHeartbeat()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.sequence = self.heartbeat_sequence
            self.heartbeat_sequence += 1
            self.hb_pub.publish(msg)

        def send_service_request(self, mode_name):
            if not self.request_client.service_is_ready():
                return
            request = SetModeByName.Request()
            request.mode_name = mode_name
            future = self.request_client.call_async(request)
            future.add_done_callback(self.on_service_response)
            self.service_sent = True
            self.get_logger().info(f'SERVICE requested mode={mode_name}')

        def on_service_response(self, future):
            try:
                response = future.result()
                self.service_done = response.success
                self.get_logger().info(
                    f'SERVICE success={response.success} active={response.active_mode} '
                    f'message={response.message}'
                )
            except Exception as exc:  # noqa: BLE001 - test diagnostic path
                self.get_logger().error(f'SERVICE exception={exc}')

        def has_api_authority(self):
            return any(event['authority'] == 'API' for event in self.events)

        def on_status(self, msg):
            event = {
                'active': msg.active_mode,
                'authority': msg.authority,
                'teleop': bool(msg.teleop_input_valid),
                'heartbeat': bool(msg.api_heartbeat_valid),
                'reason': msg.last_transition_reason,
            }
            current = tuple(event.values())
            if current == self.last_event:
                return
            self.last_event = current
            self.events.append(event)
            self.get_logger().info(
                'EVENT '
                f't={self.elapsed():.2f} active={event["active"]} authority={event["authority"]} '
                f'teleop={event["teleop"]} heartbeat={event["heartbeat"]} reason={event["reason"]}'
            )

        def finish(self):
            passed, message = verify_expected_sequence(
                self.events, scenario.expected_sequence
            )
            if service_mode and not self.service_done:
                passed = False
                message = (
                    f'{message}; service request for {service_mode} did not succeed'
                )
            self.get_logger().info(f'RESULT {"PASS" if passed else "FAIL"} {message}')
            rclpy.shutdown()

    rclpy.init()
    rclpy.spin(Driver())


def rc_command_for_scenario(name, t):
    if name == 'manual_sequence':
        if t < 5.0:
            return RcCommand('DampingRequested')
        if t < 13.0:
            return RcCommand('ReadyPoseRequested')
        if t < 21.0:
            return RcCommand('VelocityRequested')
        if t < 29.0:
            return RcCommand('MimicRequested')
        return RcCommand('VelocityRequested')

    if name == 'manual_mimic_mid_retrigger':
        if t < 5.0:
            return RcCommand('DampingRequested')
        if t < 13.0:
            return RcCommand('ReadyPoseRequested')
        if t < 21.0:
            return RcCommand('VelocityRequested')
        if t < 29.0:
            return RcCommand('MimicRequested')
        if t < 31.0:
            return RcCommand('VelocityRequested')
        if t < 34.0:
            return RcCommand(None)
        return RcCommand('MimicRequested')

    if name == 'manual_ready_watchdog':
        return RcCommand('ReadyPoseRequested') if t < 7.0 else None

    if name == 'manual_velocity_watchdog':
        if t < 9.0:
            return RcCommand('ReadyPoseRequested')
        if t < 9.1:
            return RcCommand('VelocityRequested')
        return None

    if name == 'manual_mimic_watchdog':
        if t < 8.0:
            return RcCommand('ReadyPoseRequested')
        if t < 16.0:
            return RcCommand('VelocityRequested')
        if t < 23.0:
            return RcCommand('MimicRequested')
        return None

    if name == 'manual_mimic_damping_switch':
        if t < 8.0:
            return RcCommand('ReadyPoseRequested')
        if t < 16.0:
            return RcCommand('VelocityRequested')
        if t < 23.0:
            return RcCommand('MimicRequested')
        return RcCommand('DampingRequested')

    if name in ('api_heartbeat_watchdog_ready', 'api_teleop_watchdog_ready'):
        if name == 'api_teleop_watchdog_ready' and t >= 12.0:
            return None
        return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)

    if name == 'api_mimic_heartbeat_watchdog':
        if t < 12.0:
            return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)
        return RcCommand('MimicRequested', api_mode=t >= 6.0)

    if name == 'api_mimic_heartbeat_watchdog_ready_switch':
        return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)

    if name == 'api_release_handoff':
        # Enter API from ReadyPose (rising edge at t=6, API at t~9), park the switch
        # at the Velocity position, then drop the API switch at t=11 to release.
        api_mode = 6.0 <= t < 11.0
        if t < 10.0:
            return RcCommand('ReadyPoseRequested', api_mode=api_mode)
        return RcCommand('VelocityRequested', api_mode=api_mode)

    if name == 'api_damping_switch':
        # Enter API from ReadyPose, then throw the damping kill switch at t=12.
        if t < 12.0:
            return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)
        return RcCommand('DampingRequested')

    if name == 'api_mimic_release_blocked':
        # API + MimicSquat (via service). Release with the switch still at Mimic
        # (blocked, stays API), then move the switch to Velocity to release.
        if t < 14.0:
            return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)
        if t < 18.0:
            return RcCommand('MimicRequested')
        return RcCommand('VelocityRequested')

    if name == 'api_entry_rejected_then_retoggle':
        # Rising edge at t=6 lands before the heartbeat (starts t=8) -> rejected.
        # Switch stays up while heartbeat appears -> consumed edge must not re-fire.
        # Move to Velocity (still MANUAL) at t=10, then down/up retoggle at t=14/16.
        if t < 14.0:
            condition = 'ReadyPoseRequested' if t < 10.0 else 'VelocityRequested'
            return RcCommand(condition, api_mode=t >= 6.0)
        return RcCommand('VelocityRequested', api_mode=t >= 16.0)

    if name == 'api_entry_rejected_in_motion':
        # ReadyPose -> MimicSquat, then raise the API switch from a motion state ->
        # rejected. Move to Velocity (still MANUAL) to prove no entry happened.
        if t < 4.0:
            return RcCommand('ReadyPoseRequested')
        if t < 14.0:
            return RcCommand('MimicRequested', api_mode=t >= 8.0)
        return RcCommand('VelocityRequested', api_mode=True)

    if name == 'api_no_entry_on_boot_switch_high':
        # API switch up from boot (t=0). No rising edge -> no entry despite a valid
        # heartbeat. Velocity at t=10 proves MANUAL; down/up retoggle at t=14/16.
        if t < 14.0:
            condition = 'ReadyPoseRequested' if t < 10.0 else 'VelocityRequested'
            return RcCommand(condition, api_mode=True)
        return RcCommand('VelocityRequested', api_mode=t >= 16.0)

    if name == 'api_release_and_loss_same_tick':
        # API + MimicSquat (via service), then release the switch to a motion position
        # while the heartbeat stops (see should_publish_heartbeat) -> release + loss
        # together. The loss path must win and substitute Velocity.
        if t < 14.0:
            return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)
        return RcCommand('MimicRequested')

    if name == 'api_mimic_release_blocked_invalid_selector':
        # API + MimicSquat (via service). Release with the Mimic switch held but
        # the selector out of detent; motion intent must still block.
        # Then move to Velocity to release. The whole release phase must finish
        # before the ~6 s squat motion completes, because completion ends the mimic
        # state and dissolves the block on its own.
        if t < 11.0:
            return RcCommand('ReadyPoseRequested', api_mode=t >= 6.0)
        if t < 14.0:
            return RcCommand('MimicRequested', selector_valid=False)
        return RcCommand('VelocityRequested')

    raise ValueError(f'unknown scenario: {name}')


def should_publish_heartbeat(name, t):
    if name in (
        'api_heartbeat_watchdog_ready',
        'api_mimic_heartbeat_watchdog',
        'api_mimic_heartbeat_watchdog_ready_switch',
        'api_release_handoff',
        'api_damping_switch',
    ):
        return 4.0 <= t < 13.0
    if name == 'api_teleop_watchdog_ready':
        return 4.0 <= t < 18.0
    if name == 'api_mimic_release_blocked':
        # Stay valid across the release phases so the exit is a switch release,
        # never a heartbeat loss.
        return 4.0 <= t < 20.0
    if name == 'api_entry_rejected_then_retoggle':
        # Heartbeat appears only after the first (rejected) edge, then stays valid
        # so the only thing blocking entry is the consumed edge.
        return 8.0 <= t < 25.0
    if name == 'api_entry_rejected_in_motion':
        return 6.0 <= t < 20.0
    if name == 'api_no_entry_on_boot_switch_high':
        # Valid from early on so non-entry is provably the missing edge, not a
        # missing heartbeat.
        return 4.0 <= t < 25.0
    if name == 'api_release_and_loss_same_tick':
        # Heartbeat stops at t=14, the same time the switch is released, so the exit
        # is a simultaneous release + heartbeat loss.
        return 4.0 <= t < 14.0
    if name == 'api_mimic_release_blocked_invalid_selector':
        return 4.0 <= t < 20.0
    return False


def verify_expected_sequence(events, expected_sequence):
    event_index = 0
    for expected in expected_sequence:
        matched = False
        while event_index < len(events):
            if event_matches(events[event_index], expected):
                matched = True
                event_index += 1
                break
            event_index += 1
        if not matched:
            return False, f'missing expected event {expected}'
    return True, 'all expected events observed'


def event_matches(event, expected):
    checks = {
        'active': expected.active,
        'authority': expected.authority,
        'teleop': expected.teleop,
        'heartbeat': expected.heartbeat,
        'reason': expected.reason,
    }
    return all(value is None or event[key] == value for key, value in checks.items())


if __name__ == '__main__':
    main()
