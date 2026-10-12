# Carry v74 model_115800 deployment

This document records the v74 integration and its validation on 2026-10-08.
The v74 asset is `locomotion/carry/stage2_v74_model_115800`; the previous v62
asset remains in place for rollback. This change was integrated on
`feature-G_summit`; it does not import the Any2Track branch.

As of 2026-10-12, the user-configured selector **206** retains the state name
`CarryStage2V74M115800` and behavior `carry_stage2_v74_m115800`, but points to
`locomotion/carry/carry_distill_student_v9_model_4800`. That student uses 318
observations and does not consume the extra histories described below. Its
local asset disables LPF and standstill switching, preserves the existing
unlimited command slew, and retains the supplied arm gains (Kp 26.76 / Kd 1.70).
The current runtime does not parse `standstill_switching`; it does not wait for
standstill before an SA arm transition. The v74 validation below does not
establish v9 closed-loop behavior. Model and asset parameter files are ignored
by Git and must be copied separately when deploying this configuration.

## Bundle and observation contract

Source: `K1_carry_v74_115800_bundle_20261008` (README, configuration snippet,
`patches/02_dual_history_obs/dual_history_obs.patch`, and `add_observations.py`).
The existing Carry port already supplies RC channels and `mode_command`, so the
older full-file Carry patch was not reapplied over the current runtime.

ONNX input is `obs [1,1353]`, output `action [1,23]`, policy period 0.02 s.
The bundle is copied unchanged, with LPF disabled. Model SHA-256:
`cbf4bc075e0754114665e4ba86735e69f0373668d2fa009e180da93d16d3e9b2`.
All five files match the supplied MD5 manifest.

| Input indices (inclusive) | Term | History | Scale |
| --- | --- | --- | --- |
| 0–311 | Existing six walking terms | 4 frames each | Existing scales |
| 312–317 | `mode_command` | 1 frame | 1 |
| 318–432 | `applied_torque` | 5 × 23 | 0.01 |
| 433–892 | `joint_pos_rel_long` | 20 × 23 | 1 |
| 893–1352 | `applied_torque_long` | 20 × 23 | 0.01 |

The existing observation manager concatenates terms in YAML order and frames
oldest to newest. The two long terms use independent histories, seeded with the
current sensor values on policy entry. Joint positions are relative to the
policy defaults. Torque comes from `JointState.effort`, reordered by joint name
into robot order and then into policy order. Each new message zero-initializes
missing effort entries, so an absent/partial effort array cannot reuse stale
values. A robot publishing no effort will therefore produce zero torque history;
check that feedback before assessing this policy on hardware.

The MuJoCo effort feedback uses actuator force. No torque is synthesized from
position commands in this integration.

## Preserved behavior

- SA CH5 controls Carry raise/lower; SD CH8 controls bow. API entry stays on CH9.
- LPF remains off. The user's previous removal of forward/lateral/yaw command
  slew limiting is preserved; Carry maximum speed limits remain active.
- Existing mode timing, policy command blending, ArmMode selector 207, and
  UMR selectors 208–211 are unchanged.
- No motion-clock hold option is added.

## Build and deployment

Build the `ai_sapiens_sim2real` package with `--symlink-install` after copying the
new asset, so the installed share directory includes its model and parameters.
Restart the sim2real process to load the new runtime and configuration. Running
processes are not updated by changing YAML or rebuilding a binary.

Assets are Git-ignored and must be transferred separately to another machine.
For rollback, restore selector 206/state/behavior to `CarryStage2V62`,
`carry_stage2_v62`, and `locomotion/carry/stage2_v62_model_109000`. The new generic
observations can remain registered; old policies do not request them.

## Validation

- ONNX shape and asset checksums verified; all 40 synthetic hands-down inputs
  produced bitwise-identical outputs to the local v62 ONNX. Carry-mode output
  changes when the new history changes.
- Unit tests check the exact 1353-element layout, policy joint reordering, torque
  scaling, five/twenty-frame windows, entry reset, preservation of the first 318
  values, and missing/partial effort messages.
- Build and all 16 functional CTest suites passed. Five lint suites passed;
  the package-wide uncrustify check reports pre-existing style issues in
  `arm_mode.hpp`, `carry_mode.hpp`, `policy_runtime.cpp`, and `test_arm_mode.cpp`.
  The files changed for this update pass the targeted style check.
- No physical robot deployment or motion test is performed by this update.

### MuJoCo integration result

A separate headless gantry test (Fast DDS domain 221, synthetic RC input) passed:
ReadyPose → v74 → SA raise → SA lower → ReadyPose → v74 re-entry → RC timeout
→ Damping. All 340 Carry observation samples were finite, used nonzero torque
feedback, and had matching current values in the short/long torque and position
histories. Command-gain interpolation remained active (596 intermediate samples).

In a separate earlier test, switching from v74 to the existing `Velocity` policy
while suspended triggered that policy's action-limit protection:
`right_ankle_roll_joint` target 6.682469 rad exceeded the 6.283185 rad runtime
limit. This was reported by `velocity_policy`, not the v74 inference, and led to
Damping. The existing Velocity policy and its limits were not modified. This
exit path requires separate investigation; the passing re-entry test used
ReadyPose. Neither test establishes free-standing walking or bow stability.
