# Ground contact in NUSim, and what it can and cannot reproduce

The one controlled hardware result about the walk policy is that the failure is
**surface-dependent**: keyboardwalk at 0.2 m/s forward, same policy, same command, and the
robot is flat-footed and fine on carpet but progressively loses balance on the
synthetic-grass field, with tip-toe appearing during the degradation. Anything identical
across those two runs cannot be the cause — the observation contract, `MotorCmd.weight`,
loop rate, IMU convention, joint ordering. The variable is foot–ground interaction.

This document records what NUSim's ground contact actually was, what it is now, and what a
controlled surface sweep in the simulator does and does not show.

## The floor's declared friction was never the friction

`models/k1/k1_scene_robocup.xml` declares the field as `friction="0.8 0.02 0.001"`. The
feet never saw 0.8.

MuJoCo derives a contact's parameters from its two geoms. When their `priority` values are
equal it takes the **element-wise maximum** of the friction vectors. The K1 foot box geom
(`models/k1/K1_22dof.xml`) declares no friction, so it inherits MuJoCo's default of `1.0`,
and `max(1.0, 0.8) = 1.0`. Verified directly on the loaded model:

```
floor geom friction: [0.8 0.02 0.001] priority 0
left-foot contact:   mu = 1.0   solref_t = 0.001
```

`solref` came out the same way: the robot's `<default>` block sets `solref="0.001 1"`, one
model timestep, so contact was also considerably stiffer than the scene implied.

So every NUSim walk to date ran on ground grippier and harder than the scene claimed, and
grippier than anything the robot stands on. That is a sufficient explanation for why the
simulator never showed the field failure — it was never simulating the field.

## The knob

`config/simulation.yaml`:

```yaml
surface:
  enabled: true
  friction: 0.30
  solref_timeconst: 0.02
  solref_dampratio: 1.0
```

`SimCore::apply_surface_override()` sets the floor geom's `priority` to 1 so its numbers
govern the contact — which is what the mujoco_playground training scene does, its floor
being declared `priority="1"`. `solref_timeconst` is clamped up to `2 * timestep` for
solver stability. Explicit `<pair>` elements are unaffected by geom priority, so the tuned
ball-vs-floor contact in the robocup scene survives unchanged (verified: `mu = 0.5`,
`condim = 6`, `solref_t = 0.02` before and after).

Default is `enabled: false`, which leaves the historical behaviour (`mu = 1.0`) in place
rather than silently changing every existing run.

`foot_log: <path>` writes a 50 Hz CSV of per-foot normal force, centre of pressure in the
foot's own frame, and sole pitch/roll. **Centre of pressure is the tip-toe measurement**:
the sole box spans `x ∈ [-0.064, +0.116]` of the foot frame with its centre at `0.026`, so
a flat stance sits near `0.026` and a foot rolled onto its toe drives `cop_x` towards
`+0.116`.

## Sweeping it: `tools/walk_surface_sweep.py`

Running the whole stack (NUSim + the NUbots role + DDS + docker) to change one number is
not a way to answer a question about ground contact. `tools/walk_surface_sweep.py` runs
`K1WalkPolicy`'s control path — same observation assembly, same 50 Hz tick, same
`tau = kp*(q_ref - q) - kd*dq` with the gains from `K1WalkPolicy.yaml` — directly on the
NUSim MJCF with onnxruntime. It reads the contract width off the ONNX input, so it takes
both the 82-obs and 79-obs policies.

It also models the ways the deployment path differs from an exact simulator, because NUSim
publishes model state directly (exact, instantaneous, never stale) and a deployment path is
defined by what the robot gets wrong:

| flag | what it models |
|---|---|
| `--linvel-mode {true,aliased,zero}` | 82-obs only: odometry exact / arriving slower than the policy tick / dead in CUSTOM |
| `--odom-rate` | publish rate for `aliased` — at 10 Hz the difference quotient is a spike train with four zeros between spikes |
| `--obs-delay`, `--action-delay` | transport delay in 20 ms ticks, fractional, linear-interpolated |
| `--imu-noise` | gaussian sigma on gyro and projected gravity |
| `--gait-frequency` | override the free-running gait clock (training randomises `U(1.25, 1.75)`, so 1.25–1.75 is in distribution) |

## Results with the deployed 82-obs policy (`k1_walk_v1_20260723_617M.onnx`)

All runs: flat scene, 2 s standing settle, then a held forward command; `--obs-delay 1.0
--action-delay 1.0` unless stated. Single seed — the harness is deterministic, so runs near
a stability boundary are not statistically separated.

**Friction alone, at the hardware's commanded speed, changes nothing.** At `vx = 0.2 m/s`
the policy walked 20 s without falling at every friction from 1.0 down to 0.25; centre of
pressure stayed near 0.015–0.018 (flat), sole pitch under 0.2°, action saturation 0.000.
Adding zeroed odometry, 10 Hz aliased odometry, one- and two-tick observation delay and
one-tick action delay — individually and together — did not change that.

**Raise the command and the surface dependence appears.** At `vx = 0.8 m/s`:

| mu | fell | fall time | distance | mean stance sole pitch |
|---|---|---|---|---|
| 0.60 | no | — | 15.19 m | +0.6° |
| 0.50 | no | — | 15.64 m | +0.8° |
| 0.40 | no | — | 16.60 m | +1.2° |
| 0.35 | no | — | 16.23 m | +0.9° |
| 0.30 | **yes** | 18.1 s | 9.62 m | +1.1° |
| 0.25 | **yes** | 5.5 s | 1.68 m | **+4.5°** |
| 0.20 | **yes** | 12.8 s | 5.82 m | 0.0° |

Sharp boundary near `mu ≈ 0.32`, and the worst case carries a clear toe-down sole tilt.

**Soft ground destabilises it independently of friction.** At `vx = 0.5 m/s`, sweeping
`solref_timeconst` over 0.002 / 0.02 / 0.05 (rigid to moderately compliant) produced no
falls at any friction from 1.0 to 0.3. At 0.10 — genuinely soft ground — the policy fell at
`mu = 1.0` and at `mu = 0.4`, and the `mu = 0.4` fall had a mean stance sole pitch of
**+5.8°**, the strongest tip-toe signature in any run. Compliance is a second, separate
axis, and it had never been randomised in training or varied in the simulator.

**The gait-clock hypothesis is not supported, and what signal there is points the other
way.** The prediction was two-sided: if the free-running clock decouples from the feet on
a slipping surface, a slower clock (1.25) should help and a faster one (1.75) should hurt.
Training randomises `U(1.25, 1.75)`, so all three are in distribution and the test changes
nothing about correctness. Four seeds each at `mu = 0.30`, `vx = 0.8 m/s`, with
`--imu-noise 0.01` to separate the runs:

| gait_frequency | falls |
|---|---|
| 1.25 | 2 / 4 |
| 1.50 | 3 / 4 |
| 1.75 | **1 / 4** |

The predicted direction does not appear; if anything the faster clock is the more stable
one. Four seeds and differences of one to two runs are not a statistically meaningful
separation, so this does not close the question — but it removes the reason to spend
training time on the mechanism, and nothing else here supports it either (the sole-pitch
traces before each fall are flat, not drifting, which is what a per-step ratchet would
look like).

## What NUSim reproduces, and what it still does not

Reproduced: **a surface-dependent fall**. The same policy at the same command survives on
grippier or stiffer ground and falls on slidey or soft ground, and the worst cases carry
measurable toe-down sole tilt. Before the priority fix this was not reachable at all,
because the friction knob did nothing.

Not reproduced:

1. **Failure at the hardware's speed.** Hardware fell on turf at 0.2 m/s. In sim, 0.2 m/s
   survives every friction down to 0.25. The simulator's stability margin is wider than the
   robot's.
2. **The progressive character.** Per-4-second windows of stance sole pitch before each sim
   fall are flat, not drifting — `mu = 0.30` held +0.7…+0.8° for sixteen seconds and then
   fell. Hardware degrades over several steps. The sim failures are abrupt.
3. **The action regime.** Action saturation was ~0.000 in nearly every sim run and never
   above 0.016, against **14.9%** of action components past 0.99 in the moving segment of
   the hardware log. The policy is operating in a different regime on the robot than in any
   simulator configuration tested here.

Point 3 is the strongest lead and the cheapest to check. A policy saturating on hardware
and not in sim points at something that makes the real robot work harder for the same
command — the leading candidate being the torque envelope, since `K1_22dof.xml`
modification 5 deliberately raised the 12 leg forceranges to the Booster actuator catalog
(hip 68/76/38.3, knee 112, ankle 38.3 N·m) so the sim can never clamp where hardware might.
Logging `motor_state.tau_est` during a firmware-controlled walk and comparing peaks against
those numbers settles it.

## What to measure on the robot

`benchmarks/surfaces.md` is the place for it. Tilt-test a loaded foot on both surfaces and
record `mu = tan(slip angle)` — 0.25 slides at 14.0°, 0.6 at 31.0°, 1.2 at 50.2°. Until
those numbers exist, the training range and the sweep corners are bracketing guesses.
