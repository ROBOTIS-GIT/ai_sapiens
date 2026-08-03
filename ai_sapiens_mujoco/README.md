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
a policy is walking. The weld can also be reattached at the robot's current
pose without snapping it back to its initial pose.

## Container

The simulation needs the MuJoCo and GLFW libraries that `docker/Dockerfile`
installs, so it runs in a locally built image rather than the published arm64
image used on the robot. The simulation overlay also requests the NVIDIA
graphics and display capabilities so the viewer uses hardware OpenGL instead
of Mesa software rendering. The host therefore needs the NVIDIA Container
Toolkit configured for Docker. Start it with the `--sim` flag:

```bash
./docker/container.sh start --sim   # build the sim image and start the container
./docker/container.sh enter         # open a shell inside it
./docker/container.sh stop --sim    # stop and remove it
```

Inside the simulation container, `nvidia-smi` should report the host GPU. If it
does not, recreate the container with `./docker/container.sh start --sim`;
adding GPU settings to an already-running container does not update its device
request or driver-library mounts.

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

# Reattach at the robot's current pose when another hanging test is needed.
ros2 service call /mujoco_sim/gantry/attach std_srvs/srv/Trigger
```

| Service | Type | Behavior |
| --- | --- | --- |
| `/mujoco_sim/gantry/set_height` | `ai_sapiens_interfaces/srv/SetGantryHeight` | Move the hook to an absolute height (m, world frame) at `speed` m/s (`speed <= 0` selects the default 0.05 m/s). Fails while released. |
| `/mujoco_sim/gantry/attach` | `std_srvs/srv/Trigger` | Align the hook with the robot's current torso pose and activate the weld. |
| `/mujoco_sim/gantry/release` | `std_srvs/srv/Trigger` | Deactivate the weld and detach the robot. |

## Viewer

The viewer window (enabled by default) renders the scene at ~60 Hz. A
MuJoCo-native side panel keeps diagnostics and controls out of the 3D viewport.
The `Diagnostics` section shows measured render FPS, current contact count, and
the active mouse-force target. `Visualization` provides native checkboxes for
visual meshes, collision geometry, contact points, contact forces, inertia
boxes, and center of mass. The CoM view shows the whole-robot CoM as a solid
red sphere and each link's own CoM as a smaller, translucent light-blue sphere.
Link marker volume is proportional to link mass, with radius clamped to keep
both light and heavy links legible.
When the gantry scene is active, the `Gantry`
section shows its state and provides motion and attach/release buttons:

| Input | Action |
| --- | --- |
| `Raise +2 cm` / `Lower -2 cm` | Nudge the gantry hook target with the on-screen buttons. |
| `Attach robot` / `Release robot` | Toggle the gantry weld with the on-screen buttons. |
| `Up` / `Down` | Nudge the gantry hook target ±0.02 m. |
| `A` / `R` | Attach or release the gantry (same as the corresponding services). |
| `V` / `G` | Toggle robot visual meshes / collision geometry. |
| `C` / `F` | Toggle contact points / one lime normal-force line per contact point. |
| `I` / `M` | Toggle equivalent inertia boxes / the whole-robot and per-link CoM markers. Visual meshes become translucent while CoM is shown. |
| Left mouse drag | Orbit the camera. |
| Right mouse drag | Pan the camera. |
| `Ctrl` + right mouse drag | Select a body and apply an external mouse-spring force. |
| `Ctrl` + `Shift` + right mouse drag | Apply the force in the horizontal plane. |
| Scroll | Zoom. |

The selected body and perturbation-force vector are highlighted while an
external force is active. Releasing the right mouse button immediately clears
the applied force so it cannot remain latched in `xfrc_applied`.

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
