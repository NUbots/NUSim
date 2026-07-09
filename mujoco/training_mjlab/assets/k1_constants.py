"""K1 robot constants for mjlab: MjSpec loader, actuator config, ready pose.

Mirrors the pattern in mjlab's `asset_zoo/robots/unitree_g1/g1_constants.py`
(`get_spec()` / `<Robot>_ARTICULATION` / `get_<robot>_robot_cfg()` / `<ROBOT>_ACTION_SCALE`),
but loads our existing vendored MJCF (`mujoco/models/k1/K1_22dof.xml`) instead of
re-importing Booster's URDF, and uses `config/gains.yaml`'s PD gains for the actuators
(deploy parity, see PORT_DECISIONS.md D3) rather than Booster's natural-frequency-derived
gains, which are vendored below as data only (`BOOSTER_K1_MOTORS`) plus an opt-in
higher-fidelity actuator cfg (`get_k1_hifi_articulation`).

JointIndexK1 order (22 joints, matches `mujoco/config/gains.yaml` and
`mujoco/shared/k1/JointIndex.hpp`):
  0 head_yaw, 1 head_pitch,
  2 l_shoulder_pitch, 3 l_shoulder_roll, 4 l_elbow_pitch, 5 l_elbow_yaw,
  6 r_shoulder_pitch, 7 r_shoulder_roll, 8 r_elbow_pitch, 9 r_elbow_yaw,
  10 l_hip_pitch, 11 l_hip_roll, 12 l_hip_yaw, 13 l_knee, 14 l_ankle_pitch, 15 l_ankle_roll,
  16 r_hip_pitch, 17 r_hip_roll, 18 r_hip_yaw, 19 r_knee, 20 r_ankle_pitch, 21 r_ankle_roll
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import yaml

from mjlab.actuator import BuiltinPositionActuatorCfg, DcMotorActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

##
# Paths.
##

_HERE = Path(__file__).resolve().parent
MUJOCO_DIR = _HERE.parent.parent  # .../NUSim/mujoco
K1_XML: Path = MUJOCO_DIR / "models" / "k1" / "K1_22dof.xml"
GAINS_YAML: Path = MUJOCO_DIR / "config" / "gains.yaml"
assert K1_XML.exists(), K1_XML
assert GAINS_YAML.exists(), GAINS_YAML

with open(GAINS_YAML) as _f:
  _GAINS = yaml.safe_load(_f)

##
# JointIndexK1 order (22), matches gains.yaml and K1_22dof.xml body-tree/keyframe order.
##

ALL_JOINT_NAMES: tuple[str, ...] = (
  "AAHead_yaw", "Head_pitch",
  "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
  "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
  "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Knee_Pitch",
  "Left_Ankle_Pitch", "Left_Ankle_Roll",
  "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch",
  "Right_Ankle_Pitch", "Right_Ankle_Roll",
)
assert len(ALL_JOINT_NAMES) == 22

LEG_START = 10
LEG_COUNT = 12

# The 12 leg joints, in JointIndexK1 order == K1_22dof.xml body-tree (natural) order.
# mjlab's `find_joints`/`find_joints_by_actuator_names` resolve matches in natural
# (body-tree) order regardless of the order given here, so this list's order is for
# readability only, but it IS the order the resulting obs/action slices come out in,
# and it matches PolicyBackend.cpp's LEG_START=10 slice exactly (verified against
# the XML's <keyframe> body-tree order in PORT_SPEC.md ss4b).
LEG_JOINT_NAMES: tuple[str, ...] = ALL_JOINT_NAMES[LEG_START : LEG_START + LEG_COUNT]
assert LEG_JOINT_NAMES == (
  "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Knee_Pitch",
  "Left_Ankle_Pitch", "Left_Ankle_Roll",
  "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch",
  "Right_Ankle_Pitch", "Right_Ankle_Roll",
)

##
# MJCF spec loader.
##


def get_spec() -> mujoco.MjSpec:
  """Load K1_22dof.xml and strip its pre-baked <motor> actuators.

  The vendored MJCF already ships a full `<actuator>` block (direct-torque
  `<motor>` elements, one per joint, named identically to the joint, used by the
  C++ deploy sim's own PdController). mjlab's `BuiltinPositionActuatorCfg` also
  adds a `<position>` actuator per target, defaulting `actuator_name` to the
  joint name (`mjlab.utils.spec.create_position_actuator`) -- with the XML's
  `<motor>` elements left in place this collides on actuator name at spec-add
  time. So: load the spec, delete every existing actuator, and let
  `K1_ARTICULATION` (below) supply mjlab-native actuators instead. This is a
  deviation from the naive "just point spec_fn at our XML" approach the PORT_SPEC
  assumed; noted here since it wasn't obvious until actually tried against the
  installed mjlab `Entity._add_actuators()` path.

  Also adds `left_foot`/`right_foot` sites (absent from the vendored XML, which
  only has the shared `imu` site) so mjlab's site-based reward/sensor terms
  (`feet_slip`, foot-velocity terms) have something to read. Positioned at the
  foot collision box center (see the box geom's `pos` in K1_22dof.xml).
  """
  spec = mujoco.MjSpec.from_file(str(K1_XML))

  for act in list(spec.actuators):
    spec.delete(act)

  foot_site_pos = (0.026, 0.0, -0.02)
  spec.body("left_foot_link").add_site(name="left_foot", pos=foot_site_pos)
  spec.body("right_foot_link").add_site(name="right_foot", pos=foot_site_pos)

  return spec


##
# v1 actuators: BuiltinPositionActuatorCfg, stiffness/damping = gains.yaml kp/kd
# (JointIndexK1 order), effort_limit = K1_22dof.xml <motor forcerange> (D3/D8).
# Grouped by (kp, kd, forcerange) triple, which happens to align with joint type.
##

_KP = _GAINS["kp"]
_KD = _GAINS["kd"]
assert len(_KP) == 22 and len(_KD) == 22

# forcerange per joint (N*m), read from K1_22dof.xml's <actuator> block (D8: keep
# our MJCF values, not Booster's raw effort_limit table).
_FORCERANGE = {
  "AAHead_yaw": 6.0, "Head_pitch": 6.0,
  "ALeft_Shoulder_Pitch": 14.0, "Left_Shoulder_Roll": 14.0,
  "Left_Elbow_Pitch": 14.0, "Left_Elbow_Yaw": 14.0,
  "ARight_Shoulder_Pitch": 14.0, "Right_Shoulder_Roll": 14.0,
  "Right_Elbow_Pitch": 14.0, "Right_Elbow_Yaw": 14.0,
  "Left_Hip_Pitch": 30.0, "Right_Hip_Pitch": 30.0,
  "Left_Hip_Roll": 35.0, "Right_Hip_Roll": 35.0,
  "Left_Hip_Yaw": 20.0, "Right_Hip_Yaw": 20.0,
  "Left_Knee_Pitch": 40.0, "Right_Knee_Pitch": 40.0,
  "Left_Ankle_Pitch": 20.0, "Right_Ankle_Pitch": 20.0,
  "Left_Ankle_Roll": 20.0, "Right_Ankle_Roll": 20.0,
}
assert set(_FORCERANGE) == set(ALL_JOINT_NAMES)

_KP_BY_NAME = dict(zip(ALL_JOINT_NAMES, _KP))
_KD_BY_NAME = dict(zip(ALL_JOINT_NAMES, _KD))


def _group(names: tuple[str, ...]) -> BuiltinPositionActuatorCfg:
  kps = {_KP_BY_NAME[n] for n in names}
  kds = {_KD_BY_NAME[n] for n in names}
  efs = {_FORCERANGE[n] for n in names}
  assert len(kps) == 1 and len(kds) == 1 and len(efs) == 1, (names, kps, kds, efs)
  return BuiltinPositionActuatorCfg(
    target_names_expr=names,
    stiffness=kps.pop(),
    damping=kds.pop(),
    effort_limit=efs.pop(),
  )


K1_ACTUATOR_HEAD = _group(("AAHead_yaw", "Head_pitch"))
K1_ACTUATOR_ARMS = _group((
  "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
  "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
))
K1_ACTUATOR_HIP_PITCH = _group(("Left_Hip_Pitch", "Right_Hip_Pitch"))
K1_ACTUATOR_HIP_ROLL = _group(("Left_Hip_Roll", "Right_Hip_Roll"))
K1_ACTUATOR_HIP_YAW = _group(("Left_Hip_Yaw", "Right_Hip_Yaw"))
K1_ACTUATOR_KNEE = _group(("Left_Knee_Pitch", "Right_Knee_Pitch"))
K1_ACTUATOR_ANKLE = _group((
  "Left_Ankle_Pitch", "Left_Ankle_Roll", "Right_Ankle_Pitch", "Right_Ankle_Roll",
))

K1_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(
    K1_ACTUATOR_HEAD,
    K1_ACTUATOR_ARMS,
    K1_ACTUATOR_HIP_PITCH,
    K1_ACTUATOR_HIP_ROLL,
    K1_ACTUATOR_HIP_YAW,
    K1_ACTUATOR_KNEE,
    K1_ACTUATOR_ANKLE,
  ),
  soft_joint_pos_limit_factor=0.9,  # matches Booster's BOOSTER_K1_CFG (PORT_SPEC ss2c).
)

##
# Ready keyframe (config/gains.yaml `ready_pose`: legs all 0, arms tucked).
##

READY_KEYFRAME = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.555),
  joint_pos={"Left_Shoulder_Roll": -1.3, "Right_Shoulder_Roll": 1.3},
  joint_vel={".*": 0.0},
)


def get_k1_robot_cfg() -> EntityCfg:
  """Fresh K1 EntityCfg (v1: gains.yaml-gain BuiltinPositionActuatorCfg legs)."""
  return EntityCfg(
    init_state=READY_KEYFRAME,
    spec_fn=get_spec,
    articulation=K1_ARTICULATION,
  )


# 0.25 * effort_limit / stiffness -- the mjlab/booster_train action-scale convention
# (PORT_SPEC ss2b/ss3e). Vendored for documentation/opt-in use only: v1 training uses
# the flat scale=0.5 in tasks/k1_walk/env_cfg.py (D4, lockstep with
# PolicyBackend.cpp's action_scale), NOT this per-joint dict.
K1_ACTION_SCALE: dict[str, float] = {}
for _a in K1_ARTICULATION.actuators:
  assert isinstance(_a, BuiltinPositionActuatorCfg)
  assert _a.effort_limit is not None
  for _n in _a.target_names_expr:
    K1_ACTION_SCALE[_n] = 0.25 * _a.effort_limit / _a.stiffness


##
# Booster K1 motor table (PORT_SPEC ss2b), vendored as data. NOT used by the v1
# actuator cfg above (D3) -- available for the opt-in hifi cfg below and for anyone
# wanting to cross-check against the real hardware spec sheet.
##


@dataclass(frozen=True)
class BoosterJointSpec:
  effort_limit: float  # N*m, continuous/peak torque rating.
  velocity_limit: float  # rad/s, no-load speed.
  knee_point_velocity: float  # rad/s, where the flat-torque plateau ends.
  armature: float  # kg*m^2, reflected rotor inertia.


BOOSTER_E6408 = BoosterJointSpec(68.0, 14.66, 1.88, 0.0478125)  # Hip Pitch.
BOOSTER_E4315 = BoosterJointSpec(76.0, 12.57, 2.62, 0.0339552)  # Hip Roll.
BOOSTER_E4310 = BoosterJointSpec(38.3, 17.59, 7.85, 0.0282528)  # Hip Yaw.
BOOSTER_E6416 = BoosterJointSpec(112.0, 12.57, 2.09, 0.095625)  # Knee.
# Ankle: E4310-based parallel-linkage wrapper, ratios 1:1, armature doubled.
BOOSTER_ANKLE = BoosterJointSpec(38.3, 17.59, 7.85, 0.0282528 * 2.0)
BOOSTER_R14 = BoosterJointSpec(14.0, 33.51, 5.24, 0.001)  # Arms.
BOOSTER_HT4438 = BoosterJointSpec(6.0, 7.85, 10.47, 0.001)  # Neck/Head.

# joint_name -> (BoosterJointSpec, natural_freq_hz, damping_ratio), per BOOSTER_K1_CFG
# (PORT_SPEC ss2c: legs/feet natural_freq=4.0; knee damping_ratio=1.0, rest 1.5; arms/head
# use the BoosterJointCfg default natural_freq=10.0, damping_ratio=2.0).
BOOSTER_K1_MOTORS: dict[str, tuple[BoosterJointSpec, float, float]] = {
  "Left_Hip_Pitch": (BOOSTER_E6408, 4.0, 1.5), "Right_Hip_Pitch": (BOOSTER_E6408, 4.0, 1.5),
  "Left_Hip_Roll": (BOOSTER_E4315, 4.0, 1.5), "Right_Hip_Roll": (BOOSTER_E4315, 4.0, 1.5),
  "Left_Hip_Yaw": (BOOSTER_E4310, 4.0, 1.5), "Right_Hip_Yaw": (BOOSTER_E4310, 4.0, 1.5),
  "Left_Knee_Pitch": (BOOSTER_E6416, 4.0, 1.0), "Right_Knee_Pitch": (BOOSTER_E6416, 4.0, 1.0),
  "Left_Ankle_Pitch": (BOOSTER_ANKLE, 4.0, 1.5), "Right_Ankle_Pitch": (BOOSTER_ANKLE, 4.0, 1.5),
  "Left_Ankle_Roll": (BOOSTER_ANKLE, 4.0, 1.5), "Right_Ankle_Roll": (BOOSTER_ANKLE, 4.0, 1.5),
  "ALeft_Shoulder_Pitch": (BOOSTER_R14, 10.0, 2.0), "ARight_Shoulder_Pitch": (BOOSTER_R14, 10.0, 2.0),
  "Left_Shoulder_Roll": (BOOSTER_R14, 10.0, 2.0), "Right_Shoulder_Roll": (BOOSTER_R14, 10.0, 2.0),
  "Left_Elbow_Pitch": (BOOSTER_R14, 10.0, 2.0), "Right_Elbow_Pitch": (BOOSTER_R14, 10.0, 2.0),
  "Left_Elbow_Yaw": (BOOSTER_R14, 10.0, 2.0), "Right_Elbow_Yaw": (BOOSTER_R14, 10.0, 2.0),
  "AAHead_yaw": (BOOSTER_HT4438, 10.0, 2.0), "Head_pitch": (BOOSTER_HT4438, 10.0, 2.0),
}
assert set(BOOSTER_K1_MOTORS) == set(ALL_JOINT_NAMES)


def _booster_stiffness_damping(spec: BoosterJointSpec, natural_freq_hz: float, damping_ratio: float):
  omega = 2.0 * 3.14159265358979 * natural_freq_hz
  stiffness = spec.armature * omega**2
  damping = 2.0 * damping_ratio * spec.armature * omega
  return stiffness, damping


def get_k1_hifi_articulation() -> EntityArticulationInfoCfg:
  """Opt-in higher-fidelity actuator cfg (D3): Booster torque-speed curve + delay.

  Uses `DcMotorActuatorCfg` (Python-side stall-torque/no-load-speed T-N curve,
  PORT_SPEC ss3c -- the closest mjlab analog to Booster's knee-point-velocity curve,
  though it loses the flat plateau below knee_point_velocity) with Booster's
  natural-frequency-derived stiffness/damping and `delay_min_lag=2, delay_max_lag=8`
  (BOOSTER_K1_CFG's uniform min/max_delay, PORT_SPEC ss2c). NOT the v1 default (D3) --
  a trained-with-this-cfg policy assumes different PD gains than `gains.yaml` and is
  NOT a drop-in for the current C++ PolicyBackend without also updating gains.yaml and
  re-validating deploy-side stability (see gains.yaml's header comment on why ankle
  kp/kd were raised from this exact Booster seed for pure-PD standing).
  """
  # One actuator group per unique (BoosterJointSpec identity, natural_freq, damping_ratio)
  # combination so joints sharing a motor spec share one DcMotorActuatorCfg, matching the
  # BuiltinPositionActuatorCfg grouping above.
  by_key: dict[tuple, list[str]] = {}
  for name, (spec, nf, dr) in BOOSTER_K1_MOTORS.items():
    key = (id(spec), nf, dr)
    by_key.setdefault(key, []).append(name)
  actuators = []
  for (_, nf, dr), names in by_key.items():
    spec = BOOSTER_K1_MOTORS[names[0]][0]
    stiffness, damping = _booster_stiffness_damping(spec, nf, dr)
    actuators.append(
      DcMotorActuatorCfg(
        target_names_expr=tuple(names),
        stiffness=stiffness,
        damping=damping,
        effort_limit=spec.effort_limit,
        saturation_effort=spec.effort_limit,
        velocity_limit=spec.velocity_limit,
        delay_min_lag=2,
        delay_max_lag=8,
      )
    )
  return EntityArticulationInfoCfg(
    actuators=tuple(actuators),
    soft_joint_pos_limit_factor=0.9,
  )


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_k1_robot_cfg())
  viewer.launch(robot.spec.compile())
