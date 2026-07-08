# Observation / action contract for `module::Locomotion`'s PolicyBackend

**Single source of truth** for the ONNX policy interface used by
`module::Locomotion::backends::PolicyBackend` (built with `-DK1_WITH_ONNX=ON`) and
by the MJX training framework (`mujoco/training/`). Anything that trains a policy for
this sim must produce a graph matching this contract exactly.

This is the proven **base-walk** contract (booster_gym `play_mujoco.py` /
htwk-gym `deploy_base_walk`): the policy controls the **12 leg joints only**; the
2 arms-per-side are held at the ready pose and the 2 head joints are driven by the
RotateHead command. A **gait clock** is part of the observation — essential for a
periodic gait.

## ONNX graph I/O

| Tensor | Name | Shape | Type |
|---|---|---|---|
| Input | `obs` | `[1, 47]` | `float32` |
| Output | `action` | `[1, 12]` | `float32` |

`PolicyBackend` calls `Ort::Session::Run` with exactly these names/shapes.

## Leg joint order (the 12 controlled joints)

A contiguous slice of `k1sim::JointIndexK1` (`mujoco/shared/k1/JointIndex.hpp`),
indices **10–21**:

```
0 LeftHipPitch   1 LeftHipRoll   2 LeftHipYaw   3 LeftKnee   4 LeftAnklePitch   5 LeftAnkleRoll
6 RightHipPitch  7 RightHipRoll  8 RightHipYaw  9 RightKnee 10 RightAnklePitch 11 RightAnkleRoll
```

## Observation layout (47 floats)

| Offset | Count | Field | Notes |
|---|---|---|---|
| 0  | 3  | Projected gravity, body frame | world `(0,0,-1)` rotated by the inverse base quaternion; unit vector (upright ⇒ `(0,0,-1)`). |
| 3  | 3  | Angular velocity (gyro), body frame | model `angular-velocity` sensor, rad/s. |
| 6  | 3  | Command `[vx, vy, vyaw]` | body-frame planar velocity (m/s, m/s, rad/s). **Zeroed while standing** (see gating). |
| 9  | 2  | Gait clock `[cos(2πφ), sin(2πφ)]` | φ advances at `gait_frequency` per policy step. **Zeroed while standing.** |
| 11 | 12 | Leg `q − q_ready` | leg joints only, rad. |
| 23 | 12 | Leg `dq × 0.1` | leg joints only, rad/s, scaled by 0.1. |
| 35 | 12 | `last_action` | previous inference output (zero after `reset()`). |

Total `3+3+3+2+12+12+12 = 47`.

**Standing gate:** when commanded speed `‖[vx,vy,vyaw]‖ ≤ stand_threshold` (default
`0.05`), the command terms `[6:9]` and gait terms `[9:11]` are zeroed and the gait
phase is frozen at 0. Training must reproduce this gating so the deployed policy
sees the same standing observation.

## Action layout (12 floats) & application

Leg-joint target offsets. Every `inference_divisor` physics steps (default 20 →
50 Hz over the 1 kHz PD), `PolicyBackend` runs the graph; on **every** physics step
it PD-tracks (via `k1sim::PdController`, `config/gains.yaml` gains):

```
q_ref[leg_j] = q_ready[leg_j] + action_scale * action[k]     (k = 0..11, leg_j = 10+k)
q_ref[arms]  = q_ready[arms]                                  (held)
q_ref[head]  = clamp(RotateHead cmd)  + gravity feed-forward  (HeadPitch, HeadYaw)
```

`action_scale` is `config/locomotion.yaml: policy.action_scale` (default **0.5**).
**Training MUST use the same `action_scale`** — document the value used and keep the
two in sync. `gait_frequency` (default 1.5 Hz) and `stand_threshold` are also under
`policy:` in `locomotion.yaml`.

### Gait clock

`φ ∈ [0,1)`, advanced once per inference (50 Hz): `φ ← fmod(φ + dt·gait_frequency, 1)`
with `dt = inference_divisor · timestep` (= 0.02 s), while moving; frozen at 0 while
standing. The policy learns to phase its footfalls against this clock.

### Head / arms

Head (`HeadYaw`, `HeadPitch`) is **not** in the action and **not** in the obs — it's
driven straight to the RotateHead command with a gravity-comp feed-forward (kp=4 head
gains sag ~0.1 rad otherwise), identical to `KinematicBackend`. Arms are held at the
ready pose. A future policy that also owns arms/head would extend both obs and action
and this doc's dims accordingly.

## `test/unit/assets/random_policy.onnx`

A tiny deterministic **untrained** MLP `obs[1,47] → Gemm(47,64) → Relu → Gemm(64,12)
→ Tanh → action[1,12]` (seed 1234, `N(0,0.1²)` weights) — smoke-tests the ONNX
plumbing only (`test/unit/test_locomotion.cpp`'s `#ifdef K1_WITH_ONNX` case: loads,
runs 2 s under `backend: policy`, asserts finite ctrl/qpos). Regenerate with the
snippet in this repo's history; not a locomotion policy.

## Training (MJX) — see `training/README.md`

`training/policy.py` builds exactly this obs and applies exactly this action;
subclasses (`training/policies/walk.py`) only shape the reward/commands. Export the
trained actor's deterministic mean to the `obs[1,47] → action[1,12]` graph via
`training/export_onnx.py`, drop it in, set `backend: policy` + `policy.path`, build
`-DK1_WITH_ONNX=ON`.
