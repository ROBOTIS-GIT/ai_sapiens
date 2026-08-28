# ai_sapiens_mujoco

`ai_sapiens_mujoco` is a MuJoCo sim2sim simulation library for the AI Sapiens
K1. It provides the MuJoCo simulation and interactive viewer consumed by the
ros2_control `SystemInterface` plugin
(`mujoco_hardware_interface/MujocoSystem`, hosted in
`ai_sapiens_hardware_interfaces/mujoco_hardware_interface`),
which exposes the same joint, IMU, and HAT (RC/BMS/watchdog) interfaces as the
real hardware components. The upper stack — the four controllers,
`ai_sapiens_sim2real`, and everything above them — runs unchanged against the
same controllers and topics it sees on the real robot.

The hardware interface also hosts the ROS gantry service node. MuJoCo itself is
provided through the ROS 2 `mujoco_vendor` package, and GLFW is resolved through
rosdep like the other build dependencies.

A gantry is modeled as a mocap body weld-constrained to `torso_link`, so the
robot can spawn hanging, be lowered until the feet touch, and be released once
a policy is walking. Reattaching restores the floating base to the last upright
hanging pose while preserving the robot's joint positions.

## Container

`docker/Dockerfile` uses rosdep to install `mujoco_vendor` and GLFW into the
container image, so the full workspace including the simulation builds in
every container — the standard workflow is enough:

```bash
./docker/container.sh start   # start the container
./docker/container.sh enter   # open a shell inside it
./docker/container.sh stop    # stop and remove it
```

Inside the container, build the workspace once (`cb`, then `sb`), then bring the
stack up. `run_k1_tmux.sh --sim` starts the Zenoh router, the
bringup, and `ai_sapiens_sim2real` in three tmux panes:

```bash
./src/ai_sapiens/run_k1_tmux.sh --sim
```

To use a RadioMaster USB transmitter as the MuJoCo RC input:

```bash
./src/ai_sapiens/run_k1_tmux.sh --sim --radiomaster-usb
```

The joystick device defaults to `/dev/input/js0`. Override it when needed:

```bash
./src/ai_sapiens/run_k1_tmux.sh --sim --radiomaster-usb \
  --device=/dev/input/js1
```

The same option selects the DualSense joystick device:

```bash
./src/ai_sapiens/run_k1_tmux.sh --sim --dualsense \
  --device=/dev/input/js1
```

To drive the robot without any controller attached:

```bash
./src/ai_sapiens/run_k1_tmux.sh --sim --keyboard
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
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py                       # hanging from gantry
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py mujoco_gantry:=false  # flat spawn on the floor
ros2 launch ai_sapiens_bringup k1_mujoco.launch.py mujoco_viewer:=false  # headless (no window)
```

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `mujoco_viewer` | `true` | Show the MuJoCo viewer window. |
| `mujoco_gantry` | `true` | Spawn hanging from the gantry. |

## Gantry workflow

The gantry follows the same order as real robot operation: bring the stack up
with the robot hanging, lower it until the feet touch the ground, start
walking, then release the hook.

```bash
# 1. Start k1_mujoco.launch.py — the robot spawns hanging from the gantry.

# 2. Lower until the feet touch the floor.
ros2 service call /mujoco_sim/gantry/set_height ai_sapiens_interfaces/srv/SetGantryHeight "{height: 1.45, speed: 0.05}"

# 3. Start walking (e.g. select the velocity policy through ai_sapiens_sim2real).

# 4. Detach the robot once it is walking.
ros2 service call /mujoco_sim/gantry/release std_srvs/srv/Trigger

# Restore the robot to the last upright hanging pose and reattach it.
ros2 service call /mujoco_sim/gantry/attach std_srvs/srv/Trigger
```

| Service | Type | Behavior |
| --- | --- | --- |
| `/mujoco_sim/gantry/set_height` | `ai_sapiens_interfaces/srv/SetGantryHeight` | Move the hook to an absolute height (m, world frame) at `speed` m/s (`speed <= 0` selects the default 0.05 m/s). Fails while released. |
| `/mujoco_sim/gantry/attach` | `std_srvs/srv/Trigger` | Restore the floating base under the last active gantry pose, clear its velocity, and activate the weld. Joint positions are preserved. |
| `/mujoco_sim/gantry/release` | `std_srvs/srv/Trigger` | Deactivate the weld and detach the robot. |

## Viewer

The viewer window (enabled by default) renders the scene at ~60 Hz. A
MuJoCo-native side panel keeps diagnostics and controls out of the 3D viewport.
The `Diagnostics` section shows measured render FPS, current contact count, and
the active mouse-force target. `Visualization` provides native checkboxes for
visual meshes, collision geometry, contact points, contact forces, inertia
boxes, and center of mass. The CoM view shows the whole-robot CoM as a solid
red sphere and each link's own CoM as a smaller, translucent light-blue sphere.
The whole-robot marker has a 55 mm radius. Link radii span 14–48 mm by mapping
relative cube-root mass across the robot subtree, so the lightest and heaviest
links remain visibly different without making every marker oversized.
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

