# MuJoCo viewer UI

The viewer uses Dear ImGui (GLFW/OpenGL3), DejaVu Sans/Bold at 16 px, a 300 px left sidebar, a 500 px right sidebar, and a 1920 x 1080 initial window matching robotis_mujoco. The Dockerfile installs `libimgui-dev` and `fonts-dejavu-core`; install those packages with apt for a native Ubuntu build.

## GUI policy control

Inside the ai_sapiens development container:

```bash
cd ~/ros2_ws/src/ai_sapiens
./run_k1_tmux.sh --sim --gui
```

`--gui` is simulation-only and mutually exclusive with keyboard, DualSense and RadioMaster input. It selects the existing KeyboardTeleopInputPlugin with `config/teleop/gui.yaml`. The viewer publishes a sequenced 20 Hz command stream on `/mujoco_teleop/input`; it does not publish joint commands. Closing or stalling the viewer lets the existing teleop watchdog enter Damping. Other input modes continue to use their own topics. Policy buttons are disabled until the GUI topic has a subscriber and fresh MANUAL mode feedback is received.

Use **Walkready**, lower the gantry to ground contact, **Start Velocity**, then release the gantry. Velocity sliders use normalized inputs in [-1, 1], scaled by the active policy's configuration. **Zero velocity** keeps the policy running with zero command; **Stop Velocity** requests Damping.

The Mimic selector reads `selectors.mimic_selector.table` through the configured selector name in the installed K1 configuration. Each motion is shown as a full-width button, available in Velocity. The running motion is green and its **Stop** button returns to Velocity. Each mimic request includes a neutral interval so the same motion can be requested again. The velocity policy name is read from the active Velocity behavior. Installing new assets/configuration still requires rebuilding `ai_sapiens_sim2real`; the UI does not hot-load arbitrary ONNX files.

## Simulation controls

- **Reset / Backspace**: request Damping and wait for new acknowledgement before restoring the initial pose, configured hang height, weld state, simulation time, and cleared commands/forces. Policy observation history is reseeded on policy re-entry through the existing controller lifecycle. No simulation reset happens when acknowledgement times out.
- **Pause / Run / Space**: enter Damping before pausing physics; resume in Damping, with no accumulated catch-up steps.
- **Align / Ctrl+A**: restore the reference camera (azimuth 120, elevation -20, distance 3, look-at 0/0/0.5).
- **Walkready** requests ReadyPose; **Damping** stops policy control.
- Menu toggles Info & Visualization, Control, Velocity Panel and Mimic Panel. F1/F2 toggle help and info overlays.

Ground Reaction Force remains disabled; PD gain editing is not implemented. GUI reset/pause require `--gui`; camera and visualization work with other input modes.

## Validation

The simulation tests cover reset restoration and pause timing. `test_viewer_teleop` covers connection gating, command normalization, repeated mimic triggers, Damping acknowledgement before reset, pause/resume and stale status. Run in an isolated ROS domain when testing alongside another robot stack.

## Floor friction and payload

The left Physics section includes Floor friction (0.05–2.0) and Reset friction. Changes apply immediately to the floor and explicit floor pairs, or to compatible foot/sole collision geoms when pairs are absent. Unnamed K1 foot geoms are identified through their ankle-roll body. Torsional and rolling coefficients stay unchanged; Reset friction restores the cached per-geom/per-pair defaults.

The Payload (kg) field in Physics provides additional mass (0–100), Apply payload and Reset payload. The added mass is a point mass at the existing base CoM, without changing rotational inertia or adding a visible object. Model constants are recomputed using scratch data, preserving live position, velocity and time. Both model parameter overrides survive the general simulation Reset, matching the reference; use their dedicated reset buttons to restore nominal values. These edits are session-local and do not change MJCF files.

The left sidebar groups primary buttons in Simulation and friction/payload in Physics. Visualization and Help & Diagnostics start collapsed. Detailed explanations and base mass totals are available in tooltips; connection failures and pending operations remain visible.
