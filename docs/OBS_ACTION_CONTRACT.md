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
| Input | `obs` | `[1, 79]` | `float32` |
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

## Observation layout (79 floats)

| Offset | Count | Field | Notes |
|---|---|---|---|
| 0  | 3  | Angular velocity (gyro), body frame | `angular-velocity` sensor, rad/s. |
| 3  | 3  | Projected gravity, body frame | world `(0,0,-1)` rotated by the inverse base quaternion; unit vector (upright ⇒ `(0,0,-1)`). |
| 6  | 3  | Command `[vx, vy, vyaw]` | body-frame planar velocity (m/s, m/s, rad/s), passed through as-is (never zeroed). |
| 9  | 22 | `q − default_pose` | all joints, rad, relative to the training keyframe (`policy.default_pose`). |
| 31 | 22 | `dq` | all joints, rad/s, **unscaled**. |
| 53 | 22 | `last_action` | previous raw network output (zero after `reset()`). |
| 75 | 4  | Gait phase `[cos φ₀, cos φ₁, sin φ₀, sin φ₁]` | two per-foot phases; see below. |

Total `3+3+22+22+22+4 = 79`.

**No base linear velocity.** The contract used to open with three linear-velocity floats
(82 total). It does not any more — see [Linear velocity at deployment](#linear-velocity-at-deployment).
The older 82-obs checkpoints (`k1_walk.onnx`, `k1_walk_v1_20260723_617M.onnx`) are **not**
loadable against the current `K1WalkPolicy`.

**Standing gate:** when the commanded speed `‖[vx,vy,vyaw]‖ < stand_threshold`
(default `0.01`), the *observed* phase is pinned to `[π, π]` (cos = −1, sin = 0); the
command terms are not modified. The internal phase keeps advancing (deployment
reference behaviour); training re-pins its phase to `[π, π]` every step while the
command is near zero, which the policy sees identically.

## Gait phase

Two phases initialized to `[0, π]` on `reset()` (feet in anti-phase). Advanced once
per inference (50 Hz): `φᵢ ← wrap(φᵢ + 2π·dt·gait_frequency)` into `[−π, π)`, with `dt`
the **measured** wall-clock period of the policy tick, clamped to `[0.005, 0.100] s`. It
used to be a hardcoded `0.02`; `Every<50, Per<seconds>>` is best-effort on an Orin also
running YOLO, and at a true 40 Hz a nominal 1.5 Hz gait actually advances at 1.20 Hz —
outside the `U(1.25, 1.75)` training range. `gait_frequency` defaults to 1.5 Hz
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

**Removed from the actor observation.** It is a privileged quantity: `local_linvel` is a
MuJoCo sensor, and there is no measured base linear velocity on the real K1 in CUSTOM
mode. The previous contract filled it by differentiating the Booster odometry
(`rt/odometer_state`), rotating into the body frame by the odometry yaw and low-passing
the result — an estimator that has never been shown to be live on hardware in CUSTOM, and
whose "no new sample" case is indistinguishable from "zero velocity".

The training env now keeps `linvel` in `privileged_state` only, so the critic still sees
it and the actor never does. Nothing on the deployment side estimates it, which also
removes `K1WalkPolicy`'s dependency on `BoosterOdometry` entirely.

This is a removal on the grounds that the signal is unsupportable at deployment, **not** a
claim that it caused the hardware failure: substituting the command for `obs[0:3]` on the
robot did not remove the tip-toe.

## Kick policy (`K1Kick`)

Deployed by `module/skill/K1KickPolicy`. A **side-foot lateral sweep**: the robot
sweeps the ball sideways with the inside foot while keeping the support foot under the
CoM. Same 22 actions and same `default_pose` offset convention as the walk policy
(`action_scale` 1.0), so only the observation differs.

### Observation layout (80 floats)

| Offset | Count | Field | Notes |
|---|---|---|---|
| 0  | 3  | Angular velocity (gyro), body frame | rad/s. **No base linear velocity** — unlike the walk contract, the kick obs starts at the gyro. |
| 3  | 3  | Projected gravity, body frame | as walk. |
| 6  | 2  | Commanded ball velocity `[vx, vy]`, torso frame | **divided by `kick_speed_cap` (3.0)**; direction of the vector is the sweep direction, magnitude is the requested ball speed. |
| 8  | 2  | Ball position `[x, y]`, torso frame | m, from vision. |
| 10 | 4  | Gait phase `[cos φ₀, cos φ₁, sin φ₀, sin φ₁]` | same clock as walk, `gait_freq` 1.5 Hz; the kick never pins it. |
| 14 | 22 | `q − default_pose` | rad. |
| 36 | 22 | `dq` | rad/s, unscaled. |
| 58 | 22 | `last_action` | previous raw network output. |

Total `3+3+2+2+4+22+22+22 = 80`.

**Slots 6-7 changed meaning on 2026-08-08** (width unchanged, so nothing in
`K1KickPolicy` needs re-ordering). They used to carry a **unit kick direction**; they
now carry the **scaled commanded ball velocity**. A pre-2026-08-08 checkpoint loads
and runs against the new code but is silently fed a unit vector where it expects a
velocity — retrain rather than resume. The caller now owns converting "pass to a
teammate 3 m away" into a speed, using its own friction estimate, instead of the
policy inferring it.

### What training models that deployment must live with

- **Ball is the only exteroceptive input.** Everything else is IMU + encoders at the
  50 Hz control rate. Training refreshes ball xy at ~16 Hz, drops 15% of frames, and
  force-holds the last value when the kicking foot is within 16 cm of the ball
  (occlusion at strike range). Deployment should hold last value on a dropped
  detection rather than zeroing or extrapolating.
- **Latency is deliberately not modelled.** This task keeps a planted base and a
  stationary pre-contact ball, so lag barely moves the relative position; dropout is
  the real gap. If the kick is ever extended to a moving ball or a run-up, latency
  has to be added.
- **Ball xy is in the torso frame,** so a stale estimate must be forward-propagated
  through odometry into the *current* torso frame, and it depends on head pitch
  (`head_pose` is the only tilt source) — a stale or wrong tilt corrupts ball xy
  directly.
- **Actuator lag is modelled:** a per-episode 0-2 control-step (0-40 ms) delay on the
  motor targets plus a 30% per-step hold of the previous target. The walk and get-up
  envs have **no** such model.

## Training / export

Train in the mujoco_playground fork (`learning/train_jax_ppo.py
--env_name=K1JoystickFlatTerrain`; `--env_name=K1Getup` for fall recovery,
`--env_name=K1Kick` for the sweep kick), export the checkpoint with
`learning/export_k1_onnx.py` (bakes obs
normalization, takes the deterministic `tanh` action, names the output
`continuous_actions`), then drop the `.onnx` into the NUbots_K1 module data dir
(`module/skill/K1WalkPolicy/data/k1_walk.onnx` /
`module/skill/K1GetUpPolicy/data/k1_getup.onnx` /
`module/skill/K1KickPolicy/data/k1_kick.onnx`) and rebuild the role. Keep the
playground MuJoCo version in lockstep with the sim's MuJoCo (3.10.0) to avoid a
sim2sim gap.

**Domain randomization now defaults ON** (`train_jax_ppo.py`, 2026-08-08) — it used
to default off and `pod_train.sh` never passed the flag, so any walk/get-up policy
trained from that script before this date had **none**. Pass
`--nodomain_randomization` to reproduce the old behaviour. The `K1Kick` randomizer
additionally varies ball mass (±15%), ball sliding and **rolling** friction (rolling
resistance decides how far a struck ball travels, and is what differs between hard
floor and turf), and per-foot sliding friction drawn independently left vs right.
