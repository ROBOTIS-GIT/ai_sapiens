# Any2Track deployment with smooth policy transitions

This integration starts from `origin/feature-smooth-policy-transition` (`283b7dd`).
It ports the OpenTrack deployment functionality from `9189d20`, `2c34321`, and
`ba281b5`, keeping command blending in `PolicyController`. The older Any2Track
runtime's separate entry interpolation and robot-specific root configuration
are not required.

## Selected local bundles

| Selector | State | Asset relative to `assets/k1` | Reference CSV |
| --- | --- | --- | --- |
| 203 | `MimicRedRedSpecialist` | `mimic/RedRed_any2track_specialist/1007` | `redred_support_v4.csv` |
| 204 | `MimicRedRedAdapter` | `mimic/RedRed_any2track_adpater/1007` | `redred_support_v5.csv` |

The `adpater` spelling is the existing directory name. These bundles intentionally
have different reference CSVs. Keep each exported model with its own parameters
and motion. Their ankle-roll limiter is declared `embedded_in_policy_output`;
it must not be applied a second time by the deployment runtime.

Policy inference and motion playback start immediately on entry, including
while the smooth transition blends joint targets and gains. The model's
one-step reference lookahead is preserved. Command blending does not pause the
motion clock or wait for ReadyPose to settle.

## Deployment contracts

- Use `runtime_type: opentrack_anyadapter`, `opentrack_specialist`, or `opentrack`.
  ONNX inputs determine which history buffers are needed.
- A 23-joint model uses `obs [1,126]`, or `[1,132]` with pelvis orientation tracking.
- Adapter `history [1,75,H]` is channel-major, oldest first. The selected bundle
  uses H=79. It pairs pre-inference sensor states with the newly issued targets.
- Specialist has no Adapter history input. Both interfaces observe previous
  motor targets, initialized from measured joints on entry.
- Optional `velocity_estimation: {version: 1, history_length: 20}` adds
  `velocity_history [1,20,75]`, time-major, with gravity before gyro. Its current
  frame uses the previous issued targets. This is separate from Adapter history,
  which stores gyro before gravity. Match the history length to the ONNX model.
- The velocity estimator is embedded in the model. Fetch `continuous_actions`
  by name; `estimated_velocity [1,3]` is diagnostic output, not a motor command.
- Extended CSV files have named quaternion, joint-position, saved joint-velocity,
  four foot-height and root-height columns. Read columns by name. Use discrete
  reference frames and the bundle's exact FPS (50 Hz for these bundles).
- Pelvis orientation tracking uses fixed initial yaw alignment and one-step
  lookahead. Preserve observation order, scaling, quaternion conventions, joint
  mapping, and `actions.reference_residual` semantics.
- Validate the complete reference-plus-residual target before command blending.
  Legacy Mimic retains its existing interpolation and action processing.

## Command/history ordering

The control loop performs sense, decide, policy inference, command blending,
position-limit clipping and command publication in that order. The publisher
increments a command counter only after publishing a joint command. The
controller then passes `last_published` to the active runtime.

On every publication the runtime refreshes `last_motor_targets`, including
intermediate blended commands between inference ticks. A pending successful
inference commits one history frame after publication, using that final command.
Skipped/rejected inference does not append history, and a disabled publisher
cannot commit unissued policy targets. Re-entry resets buffers from the current
sensors. `last_published` records software publication, not a hardware
acknowledgment or measured actuator position.

## Build and local checks

Run in the ROS workspace/container:

```bash
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
cd /root/ros2_ws
colcon build --symlink-install --packages-select ai_sapiens_sim2real
colcon test --packages-select ai_sapiens_sim2real --event-handlers console_direct+
colcon test-result --verbose
```

`test_opentrack_runtime` runs tiny deterministic ONNX fixtures without the robot
assets. It checks Specialist and Adapter interfaces, reversed joint mapping,
published/clipped target feedback, history reset, skipped inference, invalid
sensor/action rejection, motion-clock advancement during blending, shape/FPS mismatch rejection,
and estimator output selection. `test_motion_playback` also checks pelvis
orientation conventions. Fixtures can be regenerated with their `generate.py`
using Python `onnx`; Python ONNX is not required to build or run the C++ tests.

For simulation, launch `ai_sapiens_bringup k1_mujoco.launch.py` and
`ai_sapiens_sim2real ai_sapiens_sim2real.launch.py` after sourcing the rebuilt
workspace. Select 203 or 204 via the configured teleop source. Finish the
ReadyPose movement before requesting a policy. Use separate ROS domains for
independent tests. Policy time is not synchronized to a paused MuJoCo viewer;
exit the policy before pause/reset and re-enter afterward.

## Robot deployment

Git ignores policy assets. Copy each complete selected bundle separately,
including external `.onnx.data` weights if the model has them, and verify file
hashes against the simulation machine. Build the new runtime on the robot and
adapt its own root configuration/selector numbering to these behavior entries.
Do not overwrite robot-specific settings with the entire desktop configuration.

Before hardware motion, verify pelvis IMU calibration, joint names/zero points,
actuator torque limits and 50 Hz policy timing. The YAML's `torque_limits` list
is metadata here; the hardware/controller must enforce the corresponding limits.
Local inference and a suspended MuJoCo test do not establish floor-contact or
physical-robot stability.

## Local validation (2026-10-08)

- ROS container build passed; all 20 CTest entries passed, including six new
  OpenTrack runtime tests and the lint checks.
- The earlier integration check (before restoring immediate motion playback)
  used a headless gantry simulation on a separate Fast DDS domain (219) and synthetic
  RC input: Damping → ReadyPose → Specialist → ReadyPose → Adapter → Velocity →
  Adapter → RC timeout → Damping.
- Both actual 1007 ONNX bundles ran with finite observations and commands.
  All 458 sampled previous-motor-target observations matched a nearby published
  command. 396 command samples had intermediate transition gains.
- This was a suspended integration check, not a floor-contact motion validation
  or a physical robot test. No robot deployment was performed.
