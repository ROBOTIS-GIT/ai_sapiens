# Any2Track adapter deployment

## BTS Boy With Luv (`bts_boywithluv_a2t`)

The supplied model exports `opentrack_anyadapter`: `obs` is
`[1,132]`, `velocity_history` is `[1,20,75]`, and `history` is `[1,75,79]`.
It has **no XY/path-position input**. Robot IMU quaternion and gyro supply attitude,
gravity, and angular velocity in both sim2sim and sim2real. Odometry is not required
and its loss does not interrupt this model. Redred's 134-element XY model retains
its existing pelvis-odometry input and watchdog.

Both models use the same `PolicyRuntime`, `Any2TrackPolicy`, and `OnnxInference`.
The loader validates each exported schema and packs the foot/root heights at the
correct offsets; it does not pad or fabricate an XY input for BTS.

Build and source the ROS workspace, then start MuJoCo and the BTS policy:

```bash
ros2 launch ai_sapiens_sim2real bts_boywithluv_any2track.launch.py start_simulator:=true
```

For an already-running simulator or real robot bringup, use:

```bash
ros2 launch ai_sapiens_sim2real bts_boywithluv_any2track.launch.py
```

This uses `config/k1_bts_boywithluv_a2t_config.yaml`. Select **205** for
`MimicBtsBoywithluvA2t`, after ReadyPose. The dedicated ReadyPose matches the first
CSV joint pose. The local main `k1_config.yaml` also registers selector 205 using
the existing state/behavior format; its shared ReadyPose is unchanged. The
0.1-second command transition and Velocity completion/fallback remain in use.
The reference contains 1749 frames at 50 Hz (34.96 seconds between first and last).

The deployment files are the original `exported/policy.onnx`,
`params/sim2real.yaml`, and the CSV selected by `motion` in the root config.
No generated reference files or model hashes are required. Compatible model
checkpoints can be replaced directly; restart the policy node to load them.

The existing ROS smoke test also accepts the BTS config:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 src/ai_sapiens/ai_sapiens_sim2real/scripts/run_any2track_smoke_test.py \
  --robot k1_bts_boywithluv_a2t
```

This uses synthetic ROS sensors and is not a physical balance test.

Validation on 2026-10-13: build and all 17 CTest targets passed. BTS and Redred ROS
smoke tests passed, including IMU-source checks, missing-odometry behavior, reentry,
and motion completion, with both generated reference files absent. The BTS
iteration-16000 training/Torch oracle matched 170 native steps loaded directly
from CSV with maximum input error `2.91e-7` and action/target error `3.82e-6` (tolerance
`2e-5`). The parity probe uses identical preceding targets on both sides to isolate
observation packing and inference differences from accumulated numerical drift.
Full closed-loop MuJoCo execution and physical robot execution have not been
validated for this bundle.

## Redred (`redred_a2t_gpt`)

`Cyclo-Mimic-K1-Rev1-Redred-Glopo-any2track` in `humanoid_motion` trains a
Specialist and then an Adapter. The supplied `redred_a2t_gpt/exported/policy.onnx`
is the **combined actor + adapter/history encoder + velocity estimator**, from
`20261009_115606_pipeline/adapter/model_020000.pt`. No second neural network is
needed. Keep its exported `sim2real.yaml` unchanged.

## Sim2sim and sim2real: the same pelvis odometry input

Both modes use the existing `/state_estimator/odom` pelvis XY from
`ai_sapiens_state_estimator`. Simulation does **not** substitute MuJoCo ground
truth for this input. The path is:

```text
MuJoCo / real robot → IMU + joint states → existing pelvis estimator
→ /state_estimator/odom → Any2Track XY alignment → path_position_error
```

Keep the existing estimator running in the localization container, with the same
ROS domain and middleware as the robot/policy. Use its usual estimator activation
procedure; do not start a duplicate estimator if it is already running.

After building and sourcing the ROS workspace, sim2sim can start K1 MuJoCo and
the policy together:

```bash
ros2 launch ai_sapiens_sim2real redred_any2track.launch.py start_simulator:=true
```

If MuJoCo is already running, omit `start_simulator:=true`. On the real robot,
start the existing robot bringup and estimator, then use the **same** policy launch:

```bash
ros2 launch ai_sapiens_sim2real redred_any2track.launch.py
```

The shared launch fixes the localization source to `/state_estimator/odom`
(`odom` → `pelvis`) in both modes. It never creates a ground-truth odometry
publisher or falls back to simulator position. Missing/stale pelvis odometry
prevents entry or requests Velocity. Existing teleop plugin/config overrides
remain available. Simulator viewer, gantry, and RadioMaster USB options can be
passed through when starting MuJoCo.

The dedicated config loads the Velocity fallback and Redred Any2Track only.
Selector **204** selects `MimicRedredAny2Track`. Its ReadyPose moves over three
seconds to the supplied Redred frame-zero joint pose. The existing 0.1-second
policy command transition is retained. Use ReadyPose, then request Mimic with
selector 204; Damping and Velocity retain their existing controls. For keyboard
input, pass the KeyboardTeleopInputPlugin and `config/teleop/keyboard.yaml` as
usual, and use the same root config in the keyboard UI.

The native runtime consumes:

- Joint positions/velocities in the configured robot order (mapped to policy order).
- Robot IMU quaternion and angular velocity expressed in the **pelvis** frame.
- Fresh `nav_msgs/Odometry` for pelvis XY only, with
  `header.frame_id=odom`, `child_frame_id=pelvis` by default. Override the existing
  localization frame/topic arguments to match the producer.

At each entry it aligns IMU yaw to reference frame zero (or the configured start
frame), and applies that fixed yaw rotation to odometry XY displacement from the
entry position. The IMU and odometry XY axes must share the same world heading.
Tracking errors are not recentered each tick. Roll/pitch and projected gravity
come from the **robot IMU**; estimator attitude is not used in policy observations
or histories. Invalid IMU orientation or invalid/stale localization prevents entry
or requests Velocity. Motion completion also requests Velocity.

The export explicitly describes **ground-truth, simulation-only XY training**.
This implementation supplies the same mathematical observation using measured
odometry in both sim2sim and sim2real; it does not turn the export into a hardware-validated
policy. Estimator drift/latency and physical transition balance have not been
validated on a real robot. For a non-actuating ROS startup check use the existing
`command_publisher_enabled:=false` argument.

## Deployment contract

| ONNX input | Shape | Contents |
| --- | --- | --- |
| `obs` | `[1,134]` | Ordered terms from the original YAML |
| `velocity_history` | `[1,20,75]` | Oldest → newest; gravity, gyro×0.05, joint offset, joint velocity×0.05, previous motor targets |
| `history` | `[1,75,79]` | Channels × oldest → newest; gyro×0.05, gravity, joint offset, joint velocity×0.05, resulting motor targets |

The adapter history pairs **pre-action sensors with that action's motor targets**.
At reset, both histories are filled with measured initial sensor/target values;
there is no history leakage across policy transitions. Velocity history includes
the current observation. The adapter history includes only preceding transitions.

The first inference uses reference frame **start + 1**, matching training reset.
`continuous_actions` is a 23-element tanh residual; the command is
`reference_joint_position + scale * residual`. `last_motor_targets` stores that
absolute target, not the residual or the command blended by the controller.
`estimated_velocity` is the model's auxiliary output; the graph already feeds it
into the actor. The runtime requests `continuous_actions` only.

`path_position_error` is reference-minus-robot XY rotated by **robot yaw only**.
`motion_anchor_ori_b` is the first two columns of the relative rotation matrix,
flattened in row-major order. Reference foot heights follow the training site's
order: left foot, right foot, left foot top, right foot top.

## Offline ground-truth reference check (separate from sim2sim)

The C++ `any2track_reference_check` executable is only an offline training/parity
baseline. It directly uses MuJoCo ground-truth XY and does **not** exercise the
pelvis estimator. The previously reported 34.18-second rollout and centimetre
errors below belong to this baseline, not to odometry-based closed-loop sim2sim.

```bash
# In /root/ros2_ws of the ROS container, after sourcing install/setup.bash:
ros2 run ai_sapiens_sim2real any2track_reference_check \
  src/ai_sapiens/ai_sapiens_sim2real/assets/k1/mimic/redred_a2t_gpt \
  src/ai_sapiens/ai_sapiens_description/mujoco/k1/scene.xml \
  1709 /tmp/redred_any2track_rollout.csv
```

This baseline starts at reference frame zero, uses 50 Hz inference, 200 Hz PD
control, Newton 10 iterations, Euler integration and training gains/torque limits.
Its saved CSV can be replayed with `scripts/view_any2track_rollout.py --scene
<scene.xml> --rollout <rollout.csv>` in a Python MuJoCo environment with a display.

## Reference loading

The runtime uses the existing `MotionReference` CSV reader. At startup it derives
reference velocities using central differences (one-sided at the ends), and
computes the four training foot-point heights from the installed
`ai_sapiens_description/mujoco/k1/k1.xml` model. This is offline FK of the CSV
reference, not a live MuJoCo position or an estimator input. No FK runs in the
control loop. CSV joint names are mapped to policy/model order.

The ONNX input/output shapes and exported observation/action schema are still
validated. There is no ONNX/CSV hash binding and no dependency on
`any2track_reference.bin`, `any2track_reference.yaml`, a training NPZ, or a separate
conversion command. The original deployment assets remain ignored by Git, so
copy ONNX, YAML, and the configured CSV to the target machine as usual.

## Validation performed

On 2026-10-12, using the supplied iteration-20000 model:

- Training/Torch versus native ONNX parity: 170 steps across two entries,
  covering both history windows and a changed odometry XY/yaw origin. Maximum
  input difference `1.79e-5`; maximum action/target difference `1.79e-5`.
- Offline GT baseline: full 34.18-second standalone rollout on the training scene: no fall;
  XY RMSE 0.0391 m, maximum 0.1036 m, minimum pelvis height 0.5741 m.
- Offline GT baseline: full rollout on the repository's K1 scene: no fall; XY RMSE 0.0430 m,
  maximum 0.1159 m, minimum pelvis height 0.5763 m.
- ROS synthetic-sensor smoke: three-input inference, reference timing, finite
  joint commands, missing/stale odometry fallback, coordinate reentry, reset
  targets, and finite-clip completion passed.
- All 17 CTest unit-test targets passed, including new Any2Track cases.
- ROS sensor-source regression passed with deliberately different IMU and estimator
  attitudes: estimator attitude changes leave policy observations unchanged; IMU
  tilt changes gravity/relative orientation; invalid IMU attitude blocks entry.

Run the ROS smoke test in an isolated domain (default 189):

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python3 src/ai_sapiens/ai_sapiens_sim2real/scripts/run_any2track_smoke_test.py
```

Reproduce the independent training parity oracle in the training environment:

```bash
python /path/to/ai_sapiens_sim2real/scripts/generate_any2track_parity.py \
  --humanoid-motion /path/to/humanoid_motion \
  --checkpoint /path/to/adapter/model_020000.pt \
  --output /tmp/any2track_parity.bin
# In the ROS workspace, after copying the oracle binary:
build/ai_sapiens_sim2real/any2track_parity_probe \
  src/ai_sapiens/ai_sapiens_sim2real/assets/k1/mimic/redred_a2t_gpt \
  /tmp/any2track_parity.bin
```

Closed-loop sim2sim with the actual pelvis estimator and physical sim2real
have not yet been validated; the ROS smoke test uses synthetic odometry.
