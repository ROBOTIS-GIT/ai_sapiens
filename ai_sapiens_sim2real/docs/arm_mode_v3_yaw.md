# V3 yaw ArmMode deployment (local MuJoCo)

Source bundle: `v3_yaw_ft_24999`, exported policy with 395 observations,
23 actions and a 0.02 s policy step. Keep its policy joint order and five-frame
term-major observation history. The existing joint-name mapping handles the
controller's different joint order.

Install the complete bundle at
`assets/k1/locomotion/arm_mode/v3_yaw_ft_24999/` and register behavior
`arm_mode_v3_yaw`. The local K1 selector is 207 (`ArmModeV3Yaw`, CH11 1140 us).
Assets remain ignored by Git; they must be copied separately on another machine.

The original ArmModePolicyRuntime implementation was not supplied. This adapter
uses the offset formula documented in the supplied YAML and these explicit
local deployment choices:

- A CH5 SA OFF-to-ON edge toggles the target. Releasing does not lower the arms;
  release and press again to toggle back. ON is 1950–2050 us.
- Entering the policy resets to walking/arms down. An already-held SA is ignored
  until released and pressed again.
- `observations.arm_mode.params.transition_seconds` defaults to 1.0 s for a full
  linear 0-to-1 transition. Reversing during the transition keeps the blend
  continuous. The installed YAML explicitly sets this value and SA bounds.
- The same blend ratio is provided to the `arm_mode` observation and the action
  offset: `ready + r * (hold - ready)`. Joint-relative observations retain the
  natural-walking defaults from `joint_properties`.
- Hold velocity ranges apply while raising, holding or lowering. Walking ranges
  return once fully lowered. No velocity acceleration limiter is added.
- Missing/stale RC input requests damping. No bow behavior is added.

After building, run in the local ai_sapiens container:

```bash
cd /root/ros2_ws/src/ai_sapiens
./run_k1_tmux.sh --sim --radiomaster-usb --device=/dev/input/jsN
```

Replace jsN with the currently detected RadioMaster device. Choose ReadyPose,
then selector 207 with the Mimic input (CH7=2000, CH6=2000). Test SA from OFF.
The startup log should report `arm_mode_v3_yaw (obs=395, action=23)` and a press
should log `[ArmMode] target=hold` or `target=walk`.

This adapter has not been compared with the original training runtime. A hanging
MuJoCo integration test verifies inference, toggle/blend history and RC-loss
handling; it does not establish walking stability or suitability for hardware.
