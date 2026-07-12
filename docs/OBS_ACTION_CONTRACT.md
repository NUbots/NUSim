# Observation / action contract for `module::Locomotion`'s PolicyBackend

**Single source of truth** for the ONNX policy interface used by
`module::Locomotion::backends::PolicyBackend` (built with `-DK1_WITH_ONNX=ON`) and by
the training side (the NUbots [mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground),
branch `feat/k1-training`, `K1JoystickFlatTerrain` / `K1JoystickRoughTerrain` tasks).
Anything that trains a policy for this sim must produce a graph matching this contract
exactly.

This is the playground **T1/K1 joystick** contract: a full-body velocity-tracking
policy over all **22 joints**, with a **two-foot gait phase** in the observation. The
deployment reference implementation is
`mujoco_playground/experimental/sim2sim/play_t1_joystick.py` (K1 is the same layout
with 22 joints); `PolicyBackend.cpp` mirrors it.

## ONNX graph I/O

| Tensor | Name | Shape | Type |
|---|---|---|---|
| Input | `obs` | `[1, 82]` | `float32` |
| Output | `continuous_actions` | `[1, 22]` | `float32` |

`PolicyBackend` calls `Ort::Session::Run` with exactly these names/shapes
(`PolicyBackend.cpp` `kInputNames`/`kOutputNames`). `continuous_actions` is the name
the playground brax-to-ONNX export produces
(`mujoco_playground/experimental/brax_network_to_onnx.ipynb`); the exported graph must
**bake in** brax's observation normalization and emit the deterministic action
(`tanh` of the distribution mode) — the sim applies no normalization of its own.

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
`dt = inference_divisor · timestep` (= 0.02 s). `gait_frequency` defaults to 1.5 Hz
(`config/locomotion.yaml: policy.gait_frequency`); training randomizes it per episode
over `U(1.25, 1.75)`, so deployment at 1.5 is in-distribution.

## Action layout (22 floats) & application

Full-body joint target offsets around the training keyframe. Every
`inference_divisor` physics steps (default 20 → 50 Hz over the 1 kHz PD),
`PolicyBackend` runs the graph; on **every** physics step it PD-tracks:

```
q_ref[j]    = default_pose[j] + action_scale * action_scale_joint[j] * action[j]
q_ref[head] = clamp(RotateHead cmd) + gravity feed-forward   (overrides j = 0, 1)
```

- `default_pose` = the playground `home` keyframe (arms tucked, legs crouched) —
  **not** `gains.yaml`'s ready pose. Config: `policy.default_pose`.
- `action_scale_joint[j] = 0.25 · effort_limit[j] / kp[j]` (booster_train convention,
  matching the playground env's per-joint `_action_scale`); `action_scale` is a global
  multiplier (training config `action_scale`, default 1.0). Config:
  `policy.action_scale_joint` / `policy.action_scale`.
- The PD gains are the **training-time** gains (`policy.kp` = the playground model's
  position-actuator `kp`; `policy.kd` = its joint `damping`, since the vendored
  `K1_22dof.xml` carries no joint damping) — not `gains.yaml`, which is tuned for the
  stand/kinematic controllers. Torques clamp to the model's motor `forcerange`, which
  is set to the Booster actuator catalog effort limits (see
  `models/k1/VENDORED.md` modification 5).

### Head / arms

The policy owns arms *and* head during training; at deployment the sim overwrites the
two head targets with the RotateHead command plus a gravity-comp feed-forward
(identical to `KinematicBackend`). `last_action` stays the raw network output for all
22 entries, so the policy's action history is what it expects.

## `test/unit/assets/random_policy.onnx`

A tiny deterministic **untrained** MLP `obs[1,82] → Gemm(82,64) → Relu → Gemm(64,22)
→ Tanh → continuous_actions[1,22]` (seed 1234, `N(0,0.1²)` weights) — smoke-tests the
ONNX plumbing only (`test/unit/test_locomotion.cpp`'s `#ifdef K1_WITH_ONNX` case:
loads, runs 2 s under `backend: policy`, asserts finite ctrl/qpos). Regenerate with
`test/unit/assets/make_random_policy.py`; not a locomotion policy.

## Training / export

Train in the mujoco_playground fork (`learning/train_jax_ppo.py
--env_name=K1JoystickFlatTerrain --domain_randomization`), export the actor with the
brax-to-ONNX path (bake obs normalization, take the deterministic `tanh` action, name
the output `continuous_actions`), drop the `.onnx` in, set `backend: policy` +
`policy.path`, build `-DK1_WITH_ONNX=ON`. Keep the playground MuJoCo version in
lockstep with the C++ deploy MuJoCo (3.10.0) to avoid a sim2sim gap.
