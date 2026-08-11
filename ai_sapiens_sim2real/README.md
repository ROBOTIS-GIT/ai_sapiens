# ai_sapiens_sim2real

`ai_sapiens_sim2real` runs ONNX control policies on an AI Sapiens robot. It
loads and validates the mode configuration and all policy assets at startup,
selects control authority and behavior at runtime, performs policy inference in
a realtime-oriented loop, and publishes joint position/gain commands.

> [!CAUTION]
> Joint command publishing is enabled by default. Before running on hardware,
> verify the controller joint order, policy assets, ROS topics, and that the
> operator can select the configured Damping state. Use
> `command_publisher_enabled:=false` for a non-actuating startup check.

## Runtime at a glance

The package deliberately separates mode selection from policy execution:

```text
ROS callbacks
  IMU / joint states / teleop / API heartbeat / cmd_vel
                         |
                         v
                  SharedControlData
                         |
       realtime loop: sense -> decide -> act -> command
                         |        |       |         |
                         |        |       |         +-> joint command publisher
                         |        |       +-----------> PolicyController
                         |        +-------------------> ModeController
                         +----------------------------> sensor handles
```

Each control tick follows this order:

1. `sense`: copy callback data into the realtime state.
2. `decide`: select failsafe, authority, velocity source, and active behavior.
3. `act`: enter or update the selected policy runtime.
4. `command`: publish position, feedforward, stiffness, and damping commands.

A transition into a policy is applied before `act`, so the new policy enters
and infers on the same tick.

## Startup sequence

Startup intentionally validates configuration before waiting for live sensors:

1. Load and validate the root YAML.
2. Resolve every policy asset and parse every policy `sim2real.yaml`.
3. Load all ONNX models and initialize command output.
4. Emit the `[startup_preflight]` log.
5. Wait for one valid teleop plugin sample.
6. Start the realtime thread and wait for required IMU, joint-state, and
   teleop inputs.
7. Reset the controllers and begin the control loop.

This ordering makes a malformed root config, missing asset, invalid policy
config, or ONNX loading failure visible before sensor startup waits.

The mode runtime starts in the configured initial state, normally `Damping`.
Even after inputs are ready, the startup gate holds that state until the
teleop input matches either `DampingRequested` or `ReadyPoseRequested`.

## Mode model

The active mode has two independent dimensions.

### Terminology

| Term | Example | Meaning |
| --- | --- | --- |
| State | `Velocity` | A named FSM node under `state_machine.states`; owns transitions and references a behavior through `run`. |
| Behavior | `velocity_policy` | A named execution configuration under `state_behaviors`. |
| Behavior kind | `policy` | The execution category: `damping`, `posture`, `policy`, or `mimic`. |
| Authority | `MANUAL` | Who owns commands and mode selection: `MANUAL`, `API_WARMUP`, or `API`. |

### Behavior

| Kind | Purpose |
| --- | --- |
| `damping` | Hold measured joint positions with configured damping and zero stiffness. |
| `posture` | Interpolate from measured positions to a configured target posture. |
| `policy` | Run an ONNX policy using its own `sim2real.yaml`. |
| `mimic` | Run a policy with reference-motion playback; internally a policy behavior. |

States and transitions are defined under `state_machine`. A concrete state
references one entry under `state_behaviors` through its `run` field. Abstract
states can provide inherited transitions but cannot become active.

### Authority

| Authority | Velocity owner | Mode selection |
| --- | --- | --- |
| `MANUAL` | Teleop input | Teleop FSM transitions |
| `API_WARMUP` | Zero velocity | Holds the current state |
| `API` | `/cmd_vel` | Mode request service |

API entry requires all of the following:

- a fresh rising edge of the configured API switch;
- a valid, advancing API heartbeat;
- an active state listed in `authority.api_entry.allowed_from_states`;
- teleop velocity within `velocity_neutral_threshold`;
- commanded API velocity within `velocity_neutral_threshold`;
- completion of the configured warmup duration.

Holding the switch high after a rejected entry does not retry. Toggle it off
and on after correcting the rejection. During an API session, heartbeat loss
takes priority over a normal switch release and performs the conservative
manual handoff.

`ApiHeartbeat.sequence` must keep advancing. Repeated messages with an
unchanged sequence become stale even if they continue arriving.

## Decision and safety priority

`ModeController::decide()` applies the system-wide priority below; the failsafe
checks themselves live in `failsafe_reason()` and the authority arbitration in
`compute_authority_change()`. The first matching condition wins:

1. teleop input timeout;
2. teleop Damping input;
3. internal Damping request;
4. policy action-limit violation;
5. bad orientation;
6. startup gate;
7. authority transition;
8. velocity source and behavior-state request.

Every failsafe in the first five entries selects the configured initial state,
which is expected to be Damping. Action-limit and bad-orientation checks apply
only while a policy behavior is active.

## Configuration

The launch file resolves `robot:=k1` to
`config/k1_config.yaml`. The root configuration connects the following
sections:

| Section | Responsibility |
| --- | --- |
| `robot_joint_order` | Command and feedback order shared by the runtime. |
| `policy_asset_roots` | Search roots for named policy assets. |
| `teleop_input` | Plugin, plugin YAML, and watchdog timeout. |
| `teleop_conditions` | Semantic input conditions such as Damping or Velocity. |
| `selectors` | Selector-code to concrete-state mappings. |
| `state_machine` | Initial state, states, parents, and transitions. |
| `state_behaviors` | Damping, posture, policy, and mimic execution details. |
| `authority` | API entry checks, warmup, and default velocity state. |
| `mimic_defaults` | Optional shared defaults for mimic playback. |

The reference configuration is
[`config/k1_config.yaml`](config/k1_config.yaml), and its Radiomaster mapping
is [`config/teleop/radiomaster_pocket.yaml`](config/teleop/radiomaster_pocket.yaml).
PC-only operation can instead use
[`config/teleop/dualsense.yaml`](config/teleop/dualsense.yaml) or
[`config/teleop/keyboard.yaml`](config/teleop/keyboard.yaml) without changing
the robot configuration.

### Policy assets

An `asset` entry is resolved below each `policy_asset_roots` directory:

```text
<asset>/
  exported/
    policy.onnx
  params/
    sim2real.yaml
    <mimic motion>.csv       # mimic only
```

For example:

```yaml
policy_asset_roots:
  - ../assets/k1

state_behaviors:
  velocity_policy:
    kind: policy
    asset: locomotion/velocity/walk_default
```

This resolves to:

```text
assets/k1/locomotion/velocity/walk_default/exported/policy.onnx
assets/k1/locomotion/velocity/walk_default/params/sim2real.yaml
```

Use `policy_path` and `sim2real_yaml_path` together when the standard asset
layout is not suitable.

## Build

ROS 2 Jazzy, the workspace package dependencies, and
[ONNX Runtime](https://github.com/microsoft/onnxruntime/releases) are
required. Install ONNX Runtime under `/usr/local`, or pass
`--cmake-args -DONNXRUNTIME_ROOT=<extracted directory>` to `colcon build`.

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to ai_sapiens_sim2real
source install/setup.bash
```

## Run

Start the ROS joystick driver and verify the controller's actual axis/button
indices before enabling command output:

```bash
ros2 run joy joy_node --ros-args -p device_id:=0 -p deadzone:=0.0
ros2 topic echo /joy
```

Linux mappings can vary with the driver and USB/Bluetooth connection mode. If
they differ from the mapping documented in `config/k1_config.yaml`, update
`config/teleop/dualsense.yaml`. The Joy driver deadzone is disabled because the
DualSense plugin applies its configured deadzone once and renormalizes the
remaining stick travel to `[-1, 1]`.

To start the joystick driver and the live DualSense dashboard together:

```bash
ros2 run ai_sapiens_sim2real run_dualsense_teleop.sh
```

The dashboard shows joystick freshness, controller readiness, active mode and
authority, the current request and mimic selection, normalized X/Y/Yaw gauges,
and the configured button map. `run_k1_tmux.sh` starts this dashboard in its
bottom-right pane.

Start with command output disabled when validating a new configuration:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py \
  robot:=k1 \
  command_publisher_enabled:=false
```

After hardware, controllers, inputs, and policies have been verified:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py robot:=k1
```

### Keyboard teleop

Build and source the workspace, then start the complete tmux layout with an
interactive keyboard pane:

```bash
./run_k1_tmux.sh --keyboard
```

The keyboard publisher starts in Damping, publishes an advancing sequence at
20 Hz, and restores the terminal when it exits. If the pane or publisher stops,
the existing teleop watchdog marks the input unavailable. Velocity values are
latched and adjusted in 0.2 normalized increments; press Space or select a mode
to zero them. Mimic is emitted as a momentary request and then returns to a
neutral request, so selecting another motion and pressing `4` creates a new
edge-triggered Mimic request.

| Key | Action |
| --- | --- |
| `1` / `2` / `3` / `4` | Request Damping / ReadyPose / Velocity / selected Mimic. |
| `W` / `S` | Increase/decrease normalized forward velocity. |
| `A` / `D` | Increase/decrease normalized left velocity. |
| `Q` / `E` | Increase/decrease normalized yaw velocity. |
| `Space` | Zero all velocity axes. |
| `Left` / `Right` | Cycle the persistent mimic selector. |
| `P` | Toggle API/Manual authority request; entering API zeros velocity. |
| `H` | Reprint the current state and key map. |
| `Ctrl-C` | Exit and restore the terminal. |

The current authority, mode request, selected mimic, velocity, and complete key
map are redrawn after every accepted key press. The terminal dashboard groups
mode, movement, motion selection, and system keys; normalized velocity is shown
both numerically and on a centered bar. The Y and Yaw bars follow spatial
direction, so a left command moves their marker left while the numeric value
continues to show the published command convention. ANSI colors are disabled
automatically for a dumb terminal or when the `NO_COLOR` environment variable
is set.

For manual startup without tmux:

```bash
ros2 run ai_sapiens_sim2real keyboard_teleop_node

KEYBOARD_TELEOP_CONFIG="$(
  ros2 pkg prefix --share ai_sapiens_sim2real
)/config/teleop/keyboard.yaml"
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py \
  teleop_input_plugin:=ai_sapiens_sim2real/KeyboardTeleopInputPlugin \
  teleop_input_config_path:="${KEYBOARD_TELEOP_CONFIG}"
```

Useful launch arguments include:

| Argument | Default | Meaning |
| --- | --- | --- |
| `robot` | `k1` | Selects `config/<robot>_config.yaml`. |
| `imu_topic` | `/imu_sensor_broadcaster/imu` | Required IMU input. |
| `joint_states_topic` | `/joint_states` | Required joint feedback. |
| `control_rate` | `1000.0` | Command loop rate in Hz. |
| `wait_for_ready_timeout` | `30.0` | Input startup timeout; `0` disables it. |
| `joint_command_topic` | `/joint_group_impedance_controller/commands` | Command output. |
| `command_publisher_enabled` | `true` | Enables hardware command publishing. |
| `enforce_position_limits` | `false` | Applies active-policy position limits. |
| `thread_priority` | `50` | Requested FIFO realtime priority. |
| `lock_memory` | `true` | Requests memory locking for the RT thread. |
| `api_heartbeat_topic` | `/ai_sapiens/api_heartbeat` | API heartbeat input. |
| `api_heartbeat_timeout` | `0.2` | Advancing-sequence watchdog timeout. |
| `cmd_vel_topic` | `/cmd_vel` | API velocity command input. |
| `teleop_input_plugin` | empty | Optional plugin override; empty uses `k1_config.yaml`. |
| `teleop_input_config_path` | empty | Optional plugin-YAML override used with the plugin override. |
| `set_mode_by_name_service` | `/ai_sapiens/set_mode_by_name` | Set mode service. |
| `status_log_enabled` | `false` | Enables periodic control-loop status logs. |
| `detailed_status_log` | `false` | Adds sensor/action previews to status logs. |
| `debug_publish_enabled` | `false` | Enables observation and action debug topics. |

Run the following to see the complete launch interface:

```bash
ros2 launch ai_sapiens_sim2real ai_sapiens_sim2real.launch.py --show-args
```

## ROS interfaces

Default runtime interfaces are:

| Direction | Name | Type |
| --- | --- | --- |
| Input | `/imu_sensor_broadcaster/imu` | `sensor_msgs/msg/Imu` |
| Input | `/joint_states` | `sensor_msgs/msg/JointState` |
| Input | `/joy` | `sensor_msgs/msg/Joy` |
| Input | `/ai_sapiens/api_heartbeat` | `ai_sapiens_interfaces/msg/ApiHeartbeat` |
| Input | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Output | `/joint_group_impedance_controller/commands` | `ai_sapiens_interfaces/msg/JointImpedanceCommand` |
| Output | `/ai_sapiens/mode_status` | `ai_sapiens_interfaces/msg/ModeStatus` |
| Service | `/ai_sapiens/set_mode_by_name` | `ai_sapiens_interfaces/srv/SetModeByName` |
| Service | `/ai_sapiens/list_modes` | `ai_sapiens_interfaces/srv/ListModes` |

The mode request service accepts all reachable concrete states under full `API`
authority while the API switch is held and the heartbeat is valid. Under
`MANUAL` authority it accepts only mimic states while the velocity/mimic switch
is in its middle position (CH6=`1500`, `ManualMimicServiceAllowed`, input code
`5`).
Moving that switch to velocity immediately overrides a service-started mimic;
moving it to mimic leaves the running motion untouched. `ListModes` reports all
concrete states and the subset currently available to the service. The legacy
`ModeStatus.api_request_available` field is also true during this manual-service
window when at least one mimic state is reachable.

AI Sapiens-specific messages and services are defined in the shared
`ai_sapiens_interfaces` package rather than this runtime package.

When `debug_publish_enabled` is true, the node also publishes:

- `/policy_input/raw_observation`;
- `/policy_output/processed_action`.

## Testing

Build with tests enabled and run the package suite:

```bash
colcon build --packages-up-to ai_sapiens_sim2real \
  --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ai_sapiens_sim2real
colcon test-result --verbose
```

The suite covers configuration validation, state-machine and startup gates,
failsafe and authority priority, API/mimic handoffs, request-gate concurrency,
action helpers, gait timing, and motion playback.

### Mode transition smoke tests

[`scripts/run_mode_smoke_tests.py`](scripts/run_mode_smoke_tests.py) runs
full-stack smoke scenarios. Each scenario starts a Zenoh RMW router, the
`ai_sapiens_bringup` stack with `use_mock_hardware:=true`, and the sim2real
node, then drives fake RC input and API heartbeats and verifies the resulting
`ModeStatus` sequence: active state, authority, and transition reason.
Scenarios cover the manual FSM path, teleop watchdog failsafes, and API
warmup, release, and heartbeat-loss handoffs.

Run inside the ROS container after sourcing the workspace (requires
`ai_sapiens_bringup` and `rmw_zenoh_cpp`):

```bash
python3 scripts/run_mode_smoke_tests.py                  # all scenarios
python3 scripts/run_mode_smoke_tests.py \
  --scenario manual_sequence --scenario api_teleop_watchdog_ready
```

The script runs every selected scenario, exits non-zero if any expected
sequence did not match, and writes per-scenario `zenoh.log`, `bringup.log`,
`driver.log`, and `sim2real.log` under `/tmp/ai_sapiens_mode_smoke/<scenario>/`.

## Code map

Recommended reading order:

1. [`src/sim2real_node.cpp`](src/sim2real_node.cpp): startup and component wiring.
2. [`src/control_loop.cpp`](src/control_loop.cpp): realtime loop and stage order.
3. [`src/controllers/mode_controller.cpp`](src/controllers/mode_controller.cpp):
   safety, authority, velocity, and state decision priority.
4. [`src/controllers/policy_controller.cpp`](src/controllers/policy_controller.cpp):
   policy loading, entry, and per-tick execution.
5. [`src/policy/policy_runtime.cpp`](src/policy/policy_runtime.cpp): observation,
   ONNX inference, and action processing.
6. [`src/config/root_config.cpp`](src/config/root_config.cpp): root schema and
   asset resolution.
7. [`src/config/sim2real_config.cpp`](src/config/sim2real_config.cpp): per-policy
   joint, observation, and action configuration.

The `mode_runtime` directory contains focused state-machine, authority runtime,
and startup helpers.
`SharedControlData` is the explicit data boundary between callbacks, mode
selection, policy execution, and command publication.
