# Observation / action contract for the K1 walk policy

**Single source of truth** for the walk-policy ONNX interface shared by the deployment
side (NUbots_K1 `module/skill/K1WalkPolicy` — OpenVINO inference streaming LowCmd
joint targets over `rt/joint_ctrl`; the sim only PD-tracks those in CUSTOM mode) and
the training side (the NUbots
[mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground), branch
`feat/k1-training`, `K1JoystickFlatTerrain` / `K1JoystickRoughTerrain` tasks).
Anything that trains a walk policy for the K1 must produce a graph matching this
contract exactly.

This is the playground **T1/K1 joystick** contract: a full-body velocity-tracking
policy over all **22 joints**, with a **two-foot gait phase** in the observation. The
deployment reference implementation is
`mujoco_playground/experimental/sim2sim/play_t1_joystick.py` (K1 is the same layout
with 22 joints); `K1WalkPolicy.cpp` mirrors it.

The **get-up policy** (`K1Getup` task, deployed by `module/skill/K1GetUpPolicy`) uses
a smaller, fully deployable observation — gyro(3), projected gravity(3),
`q − default_pose`(22), `dq`(22), `last_action`(22) = **72** — and its 22 actions are
offsets on the **current** joint configuration (not the home pose), scaled by the same
per-joint scale times the training `action_scale` (**3.0** — get-up needs ~75% of the
actuator torque limits). Everything else below (joint order, export path, output tensor
name) applies to it unchanged.

## ONNX graph I/O

| Tensor | Name | Shape | Type |
|---|---|---|---|
| Input | `obs` | `[1, 82]` | `float32` |
| Output | `continuous_actions` | `[1, 22]` | `float32` |

`K1WalkPolicy` runs the graph with exactly these names/shapes (OpenVINO, CPU).
`continuous_actions` is the name the playground brax-to-ONNX export produces
(`learning/export_k1_onnx.py`); the exported graph must **bake in** brax's observation
normalization and emit the deterministic action (`tanh` of the distribution mode) —
the deployment side applies no normalization of its own.

## Joint order (all 22 joints)

`k1sim::JointIndexK1` (`mujoco/shared/k1/JointIndex.hpp`), which is identical to the
playground `k1_mjx_feetonly.xml` actuator order:

```
 0 HeadYaw            1 HeadPitch
 2 LeftShoulderPitch  3 LeftShoulderRoll  4 LeftElbowPitch  5 LeftElbowYaw
 6 RightShoulderPitch 7 RightShoulderRoll 8 RightElbowPitch 9 RightElbowYaw
10 LeftHipPitch  11 LeftHipRoll  12 LeftHipYaw  13 LeftKneePitch  14 LeftAnklePitch  15 LeftAnkleRoll
16 RightHipPitch 17 RightHipRoll 18 RightHipYaw 19 RightKneePitch 20 RightAnklePitch 21 RightAnkleRoll
```

## Observation layout (82 floats)

| Offset | Count | Field | Notes |
|---|---|---|---|
| 0  | 3  | Base linear velocity, body frame | `linear-velocity` velocimeter at the `imu` site (training: `local_linvel` sensor). |
| 3  | 3  | Angular velocity (gyro), body frame | `angular-velocity` sensor, rad/s. |
| 6  | 3  | Projected gravity, body frame | world `(0,0,-1)` rotated by the inverse base quaternion; unit vector (upright ⇒ `(0,0,-1)`). |
| 9  | 3  | Command `[vx, vy, vyaw]` | body-frame planar velocity (m/s, m/s, rad/s), passed through as-is (never zeroed). |
| 12 | 22 | `q − default_pose` | all joints, rad, relative to the training keyframe (`policy.default_pose`). |
| 34 | 22 | `dq` | all joints, rad/s, **unscaled**. |
| 56 | 22 | `last_action` | previous raw network output (zero after `reset()`). |
| 78 | 4  | Gait phase `[cos φ₀, cos φ₁, sin φ₀, sin φ₁]` | two per-foot phases; see below. |

Total `3+3+3+3+22+22+22+4 = 82`.

**Standing gate:** when the commanded speed `‖[vx,vy,vyaw]‖ < stand_threshold`
(default `0.01`), the *observed* phase is pinned to `[π, π]` (cos = −1, sin = 0); the
command terms are not modified. The internal phase keeps advancing (deployment
reference behaviour); training re-pins its phase to `[π, π]` every step while the
command is near zero, which the policy sees identically.

## Gait phase

Two phases initialized to `[0, π]` on `reset()` (feet in anti-phase). Advanced once
per inference (50 Hz): `φᵢ ← wrap(φᵢ + 2π·dt·gait_frequency)` into `[−π, π)`, with
`dt = 0.02 s` (the 50 Hz inference tick). `gait_frequency` defaults to 1.5 Hz
(`K1WalkPolicy.yaml: gait_frequency`); training randomizes it per episode over
`U(1.25, 1.75)`, so deployment at 1.5 is in-distribution.

## Action layout (22 floats) & application

Full-body joint target offsets around the training keyframe. Every 50 Hz tick
`K1WalkPolicy` runs the graph and emits a LowCmd whose per-motor targets the
robot/sim PD-tracks at its own rate:

```
q_ref[j]    = default_pose[j] + action_scale * action_scale_joint[j] * action[j]
q_ref[head] = clamp(latest BoosterHeadRot)   (overrides j = 0, 1)
```

- `default_pose` = the playground `home` keyframe (arms tucked, legs crouched) —
  **not** `gains.yaml`'s ready pose. Config: `K1WalkPolicy.yaml: default_pose`.
- `action_scale_joint[j] = 0.25 · effort_limit[j] / kp[j]` (booster_train convention,
  matching the playground env's per-joint `_action_scale`); `action_scale` is a global
  multiplier (training config `action_scale`, walk default 1.0). Config:
  `action_scale_joint` / `action_scale`.
- The PD gains sent in the LowCmd are the **training-time** gains (`kp` = the
  playground model's position-actuator `kp`; `kd` = its joint `damping`, since the
  vendored `K1_22dof.xml` carries no joint damping). The sim clamps the resulting
  torque to the model's motor `forcerange`, which is set to the Booster actuator
  catalog effort limits (see `models/k1/VENDORED.md` modification 5).

### Head / arms

The policy owns arms *and* head during training; at deployment `K1WalkPolicy`
overwrites the two head LowCmd targets with the latest `BoosterHeadRot` command
(configurable head gains, no gravity feed-forward). `last_action` stays the raw
network output for all 22 entries, so the policy's action history is what it expects.

### Linear velocity at deployment

Training observes the privileged `local_linvel` sensor. The deployment side has no
such sensor: `K1WalkPolicy` differentiates the Booster odometry (`rt/odometer_state`),
rotates it into the body frame by the odometry yaw and low-passes it
(`linvel_alpha`); the z component is unobservable and sent as 0. Verified stable
walking in NUSim with this estimate.

## Training / export

Train in the mujoco_playground fork (`learning/train_jax_ppo.py
--env_name=K1JoystickFlatTerrain --domain_randomization`; `--env_name=K1Getup` for
fall recovery), export the checkpoint with `learning/export_k1_onnx.py` (bakes obs
normalization, takes the deterministic `tanh` action, names the output
`continuous_actions`), then drop the `.onnx` into the NUbots_K1 module data dir
(`module/skill/K1WalkPolicy/data/k1_walk.onnx` /
`module/skill/K1GetUpPolicy/data/k1_getup.onnx`) and rebuild the role. Keep the
playground MuJoCo version in lockstep with the sim's MuJoCo (3.10.0) to avoid a
sim2sim gap.
