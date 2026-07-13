# Vendored assets: Booster K1 MJCF

## Source

- Repository: https://github.com/BoosterRobotics/booster_assets
- License: BSD-3-Clause (see `LICENSE.booster_assets`, copied verbatim from the
  repo root)
- Commit vendored: `486da0f864016676de6dab724fd60a8a551655d1`
  (author date 2025-12-05T06:52:08Z; latest commit touching `robots/K1` on
  `main` as of the vendoring date below, per
  `GET /repos/BoosterRobotics/booster_assets/commits?path=robots/K1&sha=main`)
- Date vendored: 2026-07-07
- Files vendored:
  - `robots/K1/K1_22dof.xml` -> `mujoco/models/k1/K1_22dof.xml` (modified, see
    below, and see the file's own top-of-file comment)
  - `robots/K1/meshes/*.STL` referenced by `K1_22dof.xml` (24 files) ->
    `mujoco/models/k1/meshes/*.STL` (byte-identical, unmodified)

Only the meshes actually referenced by `K1_22dof.xml` were vendored (not the
full `robots/K1/meshes/` directory, which also contains meshes for the
`-ZED`/`_locomotion` URDF variants we don't use, e.g. `Head_2_ZED.STL`,
`Left_Foot_old.STL`, `L1_*_Ball.STL` crank-linkage meshes, etc.).

## Local modifications to `K1_22dof.xml`

(Also listed in a comment at the top of the file itself.)

1. **Freejoint root.** Upstream already rooted the `Trunk` body with
   `<joint name="world_joint" type="free" .../>` (functionally a free
   joint). Converted to the canonical `<freejoint name="root"/>` shorthand
   for clarity; no behavioral change (qpos/qvel layout is identical either
   way: 7 qpos / 6 qvel for the base).
2. **Removed scene dressing.** Upstream's `<worldbody>` included a skybox
   texture, a checker "texplane" texture + "matplane" material, a `ground`
   plane geom, and two ad-hoc lights, all as siblings of the `Trunk` body.
   These were removed so the file is a pure robot (body tree + its own
   assets/actuators/sensors/defaults) that can be `<include>`-d into a scene
   file without a duplicate floor or conflicting lighting. The robot's own
   visual/collision assets (meshes, the trunk's placeholder collision
   primitives, etc.) were kept untouched.
3. **Added sensors.** `<accelerometer name="acceleration" site="imu"
   noise="0.01"/>` and `<velocimeter name="linear-velocity" site="imu"
   noise="0.01"/>`, alongside the upstream `framequat "orientation"` and
   `gyro "angular-velocity"`.
4. **Added a `<keyframe>`** with `ready`, `lying_front`, `lying_back` (see
   below for how the numbers were derived/verified).
5. **Raised the 12 leg motor forceranges** from upstream's conservative
   values (hip pitch/roll/yaw &plusmn;30/35/20, knee &plusmn;40, ankle
   &plusmn;20 N&middot;m) to the Booster actuator catalog effort limits
   (&plusmn;68/76/38.3, &plusmn;112, &plusmn;38.3) used by booster_train and
   the mujoco_playground K1 training model (`k1_mjx_feetonly.xml`'s
   `actuatorfrcrange`). The RL walk policy is trained against the catalog
   limits; the lower upstream caps would clamp its torques and break the
   sim-to-sim transfer. Head/arm limits (&plusmn;6/&plusmn;14) already
   matched. (Also listed in the file's top-of-file comment.)

## Keyframe derivation and settle verification

The task brief specified approximate numbers (base at `(0,0,0.53)` for
`ready`; base z &asymp; 0.13 for the lying poses). Rather than using those
literally, each keyframe's base height/orientation was **derived from this
mesh's actual collision geometry** (via `m.geom_aabb` world-frame corner
projection, over all `contype!=0 or conaffinity!=0` geoms) so that the
lowest point of the body sits ~3 mm above the floor plane at z=0, then
**verified by simulation**: load the keyframe, `mj_forward`, step 500
times (0.5 s) with a floor plane present, and check the base doesn't
show an unphysical jump/explosion (which is what happens if a keyframe
starts in non-trivial floor interpenetration combined with the model's
fairly stiff default contact `solref="0.001 1"` -- confirmed by first trying
the brief's literal numbers, which penetrate the floor by ~2 cm for `ready`
and blow up in <10 ms of sim time, base briefly exceeding 1 m altitude).

Derived values (all verified to settle cleanly, no explosion, over 500 steps
= 0.5 s):

- `ready`: base `(0, 0, 0.555)`, identity quat, all joints 0 except
  `Left_Shoulder_Roll=-1.3`, `Right_Shoulder_Roll=+1.3` (as specified).
  With **zero control torque** (these are direct-torque `<motor>`
  actuators, not position servos -- see below), this pose is naturally
  unstable over ~1 s (an unactuated biped falls over under gravity, as
  expected -- there is no balance controller baked into the keyframe).
  This is unrelated to, and doesn't indicate a problem with, the
  interpenetration check: the settle test confirms no *explosive* startup
  transient, not that the robot can stand un-actuated.
- `lying_front` (prone/face-down): base `(0, 0, 0.075)`, quat
  `(0.707107, 0, 0.707107, 0)` (+90&deg; pitch about Y). Local +x is the
  robot's front/chest direction (confirmed via the `K1logo` chest-plate
  mesh's geom offset, which sits at local x&asymp;+0.073); a +90&deg;
  pitch about Y maps local +x to world -z, i.e. chest-down. Joints: modest
  hip/knee/elbow bends (0.15-0.3 rad) for a plausible "collapsed" look --
  intentionally kept small, since any bend about a joint's Y axis directly
  adds to the pitch-rotated vertical plane and large bends (originally
  tried ~1.2-1.3 rad) push the required base height up to ~0.38 m instead
  of the intended ~0.13 m by making a limb (not the trunk/chest) the
  lowest point.
- `lying_back` (supine/face-up): base `(0, 0, 0.066)`, quat
  `(0.707107, 0, -0.707107, 0)` (-90&deg; pitch about Y). Joints: small
  bends in the opposite sense.

These base heights (0.555 / 0.075 / 0.066) differ from the brief's
approximate 0.53 / 0.13 / 0.13 -- this is a deliberate correction based on
verified, non-penetrating contact with this specific mesh's actual
collision geometry, not an oversight. `check_model.py` does not assert
exact keyframe qpos values (only that the three keys exist by name), so
this doesn't conflict with the contract test.

`ctrl` is intentionally omitted from all keyframes (defaults to 0 torque):
the 22 actuators in `K1_22dof.xml` are `<motor>` (direct-torque) elements,
not position servos, so a keyframe's joint *targets* for a PD controller
are meaningless as raw `ctrl` values. Per-joint PD targets are supplied
externally at runtime (`module/Simulation` + `config/gains.yaml`, owned by
workstream B).

## Scene: `k1_scene_robocup.xml`

`<include file="K1_22dof.xml"/>` plus a procedurally-generated RoboCup
KidSize field, ported from this repo's `worlds/k1_robocup.wbt` and
`protos/robocup_field/{RobocupSoccerField,RobocupGoal}.proto` (dimensions
read directly from the proto's per-size JS table for `size == "kid"`):
field 9 x 6 m playing area + 1 m border (11 x 8 m total grass extent), goal
2.6 m wide (post-center to post-center) x 1.2 m high (post radius 0.05 m),
center circle radius 0.75 m, goal area 1 x 3 m, penalty area 2 x 5 m,
penalty mark 1.5 m from the goal line, goal line/touchline/center-line at
the field boundary and x=0 respectively.

**Texture/licensing note:** `protos/robocup_field/`'s field appearance
(`Grass.proto`, fetched externally from
`cyberbotics/webots` at `R2022b`) and the ball textures under
`protos/robocup_field/ball_textures/*.jpg` are licensed "Copyright
Cyberbotics Ltd., licensed for use only with Webots" (or reference the
external Cyberbotics repo directly) -- neither is redistributable into this
MuJoCo scene. Accordingly the pitch uses a **procedural** green checker
MuJoCo material, and line markings (touchlines, goal lines, center
line/circle/mark, goal areas, penalty areas, penalty marks) are built from
thin white `contype="0" conaffinity="0"` box geoms at the standard KidSize
positions above, per the task brief's fallback instruction. The ball is a
plain `PBRAppearance`-free solid-color sphere for the same reason.

**Ball body/geom offset (important for anyone editing this file):** the
`ball` body is declared at `pos="0 0 0"`, with its sphere geom offset to
`pos="1.38 0 0.0785"`, rather than putting that offset on the body (which
is what the task brief's literal snippet shows). This is deliberate, not
an oversight -- see the long comment at the top of `k1_scene_robocup.xml`
for the full explanation: `K1_22dof.xml`'s own keyframes have qpos sized
for the bare 29-DOF robot; once the scene adds the ball's freejoint
(model nq becomes 36), MuJoCo automatically zero-pads the 7 missing ball
qpos entries in the *inherited* keyframes to `(0,0,0,1,0,0,0)` (verified
empirically -- it is a zero pad, not a copy of `qpos0`/the body's `pos`
attribute), and MuJoCo rejects a second `<key>` element reusing the name
`ready` (also verified empirically -- it's a hard compile error, "repeated
name ... in key"), so the keyframes can't simply be redefined at the scene
level either. Keeping the ball body's own frame at the origin means the
zero-padded default *is* the correct rest pose, so `ready`/`lying_front`/
`lying_back` all place the ball correctly with no scene-level keyframe
override needed. A separate `kickoff` keyframe (new name, no conflict) is
defined in the scene for the actual RoboCup match-start layout (robot at
the Webots world's red-K1 spawn, ball still resolved via the same
zero-pad mechanism).

**Ball bounce tuning.** MuJoCo's contact model produces genuine
under-damped/bouncy contact resolution when a contact's `solref`
`dampratio` is set below 1 (default is critically damped = no bounce).
The ball/floor contact is pinned via an explicit `<contact><pair
geom1="ball" geom2="floor" .../></contact>` (rather than relying on the
default geom-level solref/solimp/friction combination rule, which:
combines friction via an element-wise **max** -- so a lower ball-only
friction attribute would have been silently overridden to the floor's
0.8 -- and combines solref/solimp via a mixing rule that empirically made
naive per-geom dampratio tuning behave very differently, and considerably
less stably, than the same numbers used in an explicit `<pair>`). Tuned by
a drop-test sweep (`solref="0.02 0.11"`, `solimp="0.9 0.95 0.001 0.5 2"`,
`friction="0.5 0.5 0.005 0.01 0.01"`): a 1 m drop's first-bounce apex
ratio measures **0.5755**, within 0.4% of the target `0.76^2 = 0.5776`
(Webots' RoboCup ball/grass bounce coefficient 0.76, ball-grass friction
0.5, general floor friction 0.8 per `worlds/k1_robocup.wbt`'s
`ContactProperties`). Subsequent bounces decay physically (each apex
lower than the last) and the ball settles to rest on the floor with no
instability. An earlier, much stiffer/more underdamped `<pair>` parameter
choice (`solref="0.002 0.2"`, chosen because it matched an isolated,
non-`<pair>` per-geom test) was found to be numerically unstable in the
explicit-`<pair>` form -- it produced a nonphysical energy-injecting
"explosion" on first impact (post-impact velocity *larger* than the
pre-impact impact velocity) -- which is why the final tuned values differ
from that first attempt; see git history / scratch notes if reproducing
this tuning.

**Ball rolling friction.** The pair's last two friction values (rolling
friction) were raised from the sweep's original `0.0001` (billiard-table:
the ball never stopped rolling) to `0.01`, which gives a natural
grass-like roll-out over a few metres. This is independent of the bounce
tuning above (restitution lives in `solref`/`solimp`); only the drop-test
friction string differs from the values recorded in the sweep notes.

Solver settings (`timestep=0.001`, `solver="Newton"`, `impratio=10`) are
inherited from `K1_22dof.xml`'s own `<option>` via `<include>` and are not
overridden at the scene level, per the task brief.
