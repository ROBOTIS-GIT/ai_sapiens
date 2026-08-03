# ai_sapiens_mujoco

`ai_sapiens_mujoco` is a MuJoCo sim2sim hardware interface for the AI Sapiens
K1. It provides a single ros2_control `SystemInterface` plugin
(`ai_sapiens_mujoco/MujocoSystem`) that owns the MuJoCo simulation and exposes
the same joint, IMU, and HAT (RC/BMS/watchdog) interfaces as the real hardware
components. The upper stack — the four controllers, `ai_sapiens_sim2real`, and
everything above them — runs unchanged against the same controllers and topics
it sees on the real robot.

A gantry is modeled as a mocap body weld-constrained to `torso_link`, so the
robot can spawn hanging, be lowered until the feet touch, and be released once
a policy is walking — mirroring real robot operation.

## Container

The simulation needs the MuJoCo and GLFW libraries that `docker/Dockerfile`
installs, so it runs in a locally built image rather than the published arm64
image used on the robot. Start it with the `--sim` flag:

```bash
./docker/container.sh start --sim   # build the sim image and start the container
./docker/container.sh enter         # open a shell inside it
./docker/container.sh stop --sim    # stop and remove it
```

Inside the container, build the workspace once (`cb`, then `sb`), then bring the
stack up. `run_k1_tmux.sh` takes the same flag and starts the Zenoh router, the
bringup, and `ai_sapiens_sim2real` in three tmux panes:

```bash
./src/ai_sapiens/run_k1_tmux.sh --sim
```

The Zenoh router matters: the container sets
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`, so without `rmw_zenohd` running the spawners
never reach the controller manager and report "Failed to acquire lock".

colcon build artifacts persist in named volumes across `stop`/`start`. If a
rebuilt image ever leaves stale artifacts behind, clear them with
`docker volume rm docker_ai_sapiens_sim_build docker_ai_sapiens_sim_install`.

## Bringup

Launch the full K1 stack against MuJoCo instead of real hardware (inside the
container, with X forwarding for the viewer):

```bash
ros2 launch ai_sapiens_bringup k1.launch.py sim_mujoco:=true                       # hanging from gantry
ros2 launch ai_sapiens_bringup k1.launch.py sim_mujoco:=true mujoco_gantry:=false  # flat spawn on the floor
ros2 launch ai_sapiens_bringup k1.launch.py sim_mujoco:=true mujoco_viewer:=false  # headless (no window)
```

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `sim_mujoco` | `false` | Run against the MuJoCo simulation instead of real hardware. |
| `mujoco_viewer` | `true` | Show the MuJoCo viewer window (sim_mujoco only). |
| `mujoco_gantry` | `true` | Spawn hanging from the gantry (sim_mujoco only). |

## Gantry workflow

The gantry follows the same order as real robot operation: bring the stack up
with the robot hanging, lower it until the feet touch the ground, start
walking, then release the hook.

```bash
# 1. Bring up with sim_mujoco:=true — the robot spawns hanging from the gantry.

# 2. Lower until the feet touch the floor.
ros2 service call /mujoco_sim/gantry/set_height ai_sapiens_interfaces/srv/SetGantryHeight "{height: 1.45, speed: 0.05}"

# 3. Start walking (e.g. select the velocity policy through ai_sapiens_sim2real).

# 4. Detach the robot once it is walking.
ros2 service call /mujoco_sim/gantry/release std_srvs/srv/Trigger
```

| Service | Type | Behavior |
| --- | --- | --- |
| `/mujoco_sim/gantry/set_height` | `ai_sapiens_interfaces/srv/SetGantryHeight` | Move the hook to an absolute height (m, world frame) at `speed` m/s (`speed <= 0` selects the default 0.05 m/s). Fails after release. |
| `/mujoco_sim/gantry/release` | `std_srvs/srv/Trigger` | Deactivate the weld and detach the robot. One-shot; further gantry commands fail. |

## Viewer

The viewer window (enabled by default) renders the scene at ~60 Hz and adds
gantry hotkeys:

| Input | Action |
| --- | --- |
| `Up` / `Down` | Nudge the gantry hook target ±0.02 m. |
| `R` | Release the gantry (same as the release service). |
| Left mouse drag | Orbit the camera. |
| Right mouse drag | Pan the camera. |
| Scroll | Zoom. |

Closing the window stops rendering only; the simulation and controllers keep
running.

## Hardware parameters

Set in
[`k1_mujoco.ros2_control.xacro`](../ai_sapiens_description/ros2_control/k1_rev1/k1_mujoco.ros2_control.xacro)
and normally driven through the launch arguments above:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `scene_file` | `scene_gantry.xml` (or `scene.xml` when `gantry` is false) | MuJoCo scene to load; missing or invalid files fail component init. |
| `viewer` | `true` | Start the GLFW viewer thread. |
| `gantry` | `true` | Spawn hanging from the gantry at `hang_height`. |
| `hang_height` | `0.90` | Pelvis height (m) at spawn when hanging. |
| `rc_channel_defaults` | `1500,1500,1500,1500,1000,1500,2000,2000,1000,1000,1000,1000,1500,1500,1500,1500` | Simulated RC channel values (CH1–CH16) reported through the `hat` sensor. |

The `rc_channel_defaults` string simulates a RadioMaster transmitter in a
neutral pose: with the default mapping
(`ai_sapiens_sim2real/config/teleop/radiomaster_pocket.yaml`), CH7=2000 with
CH6=1500 produces `input_code` 0 — no state-machine request — so
`ai_sapiens_sim2real` stays in its initial state until you drive the channels
yourself, and CH8=2000 makes `api_mode` available. The sticks (CH1/CH2/CH4)
sit at 1500, i.e. zero commanded velocity.
