"""K1 walk task env cfg: 47-obs/12-action Base_Walk contract (D1),
ParameterWalk reward math reduced to 3-command walk (D2).

Does NOT call `mjlab.tasks.velocity.velocity_env_cfg.make_velocity_env_cfg()` --
its default `actor_terms` (base_lin_vel, base_ang_vel, projected_gravity, joint_pos,
joint_vel, actions, command, height_scan) is a different obs layout with no
gait-clock term and full-body (not leg-only) joints (PORT_SPEC ss3f). We assemble the
observation group by hand to reproduce PolicyBackend.cpp::build_obs bit-for-bit.
"""

from __future__ import annotations

import math

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers.action_manager import ActionTermCfg
from mjlab.managers.command_manager import CommandTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.tasks.velocity import mdp as velocity_mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.viewer import ViewerConfig

# NOTE: imported as bare `training_mjlab`, NOT `mujoco.training_mjlab` -- `mujoco` is
# already a top-level installed package (the MuJoCo Python bindings this very file
# also imports transitively via mjlab); shadowing it with our repo's `mujoco/`
# directory would break every other consumer of the real `mujoco` package in this
# venv. `training_mjlab` is importable once `NUSim/mujoco/` (its parent, NOT
# `NUSim/` itself) is on `sys.path` -- see the smoke test / VERIFIED_API.md for how
# this repo's `./b` wiring should add it (out of scope for this port; see report).
from training_mjlab.assets.k1_constants import LEG_JOINT_NAMES, get_k1_robot_cfg
from training_mjlab.tasks.k1_walk import mdp as k1_mdp

# GYRO_SENSOR: K1_22dof.xml's <gyro name="angular-velocity" site="imu"/>, exposed by
# mjlab as "<entity_name>/<xml_sensor_name>" once attached into the scene (V4,
# confirmed against installed mjlab.scene.Scene._add_sensors -- see VERIFIED_API.md).
GYRO_SENSOR_NAME = "robot/angular-velocity"

FEET_CONTACT_SENSOR = "feet_ground_contact"
SELF_COLLISION_SENSOR = "self_collision"

# D5: fixed gait clock, not a sampled command.
GAIT_FREQUENCY = 1.5
STAND_THRESHOLD = 0.05

# D2 reward constants (ParameterWalk, PORT_SPEC ss1b).
TRACKING_SIGMA = 0.25
BASE_HEIGHT_TARGET = 0.52
TERMINATE_HEIGHT = 0.35

# ParameterWalk torque_limit == our K1_22dof.xml <motor forcerange> for the 12 leg joints
# (D8), keyed by joint/actuator name (see k1_constants._FORCERANGE).
LEG_TORQUE_LIMIT = {
  "Left_Hip_Pitch": 30.0, "Right_Hip_Pitch": 30.0,
  "Left_Hip_Roll": 35.0, "Right_Hip_Roll": 35.0,
  "Left_Hip_Yaw": 20.0, "Right_Hip_Yaw": 20.0,
  "Left_Knee_Pitch": 40.0, "Right_Knee_Pitch": 40.0,
  "Left_Ankle_Pitch": 20.0, "Right_Ankle_Pitch": 20.0,
  "Left_Ankle_Roll": 20.0, "Right_Ankle_Roll": 20.0,
}
assert set(LEG_TORQUE_LIMIT) == set(LEG_JOINT_NAMES)


def make_k1_walk_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create the K1 walk task configuration."""

  ##
  # Observations: 47-dim Base_Walk layout, in order (D1 / PORT_SPEC ss1a/ss4a).
  # [0:3] projected_gravity, [3:6] gyro, [6:11] gated command + gait clock cos/sin,
  # [11:23] leg joint_pos_rel, [23:35] leg joint_vel_rel * 0.1, [35:47] last_action.
  ##

  actor_terms = {
    "projected_gravity": ObservationTermCfg(
      func=envs_mdp.projected_gravity,
      noise=Unoise(n_min=-0.05, n_max=0.05),
    ),
    "gyro": ObservationTermCfg(
      func=envs_mdp.builtin_sensor,
      params={"sensor_name": GYRO_SENSOR_NAME},
      noise=Unoise(n_min=-0.2, n_max=0.2),
    ),
    "command_gait_clock": ObservationTermCfg(
      func=k1_mdp.k1_command_gait_clock,
      params={
        "command_name": "twist",
        "stand_threshold": STAND_THRESHOLD,
        "gait_frequency": GAIT_FREQUENCY,
      },
    ),
    "joint_pos": ObservationTermCfg(
      func=envs_mdp.joint_pos_rel,
      params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES)},
      noise=Unoise(n_min=-0.01, n_max=0.01),
    ),
    "joint_vel": ObservationTermCfg(
      func=envs_mdp.joint_vel_rel,
      params={"asset_cfg": SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES)},
      noise=Unoise(n_min=-1.5, n_max=1.5),
      scale=0.1,
    ),
    "actions": ObservationTermCfg(func=envs_mdp.last_action),
  }

  observations = {
    "actor": ObservationGroupCfg(
      terms=actor_terms,
      concatenate_terms=True,
      enable_corruption=True,
    ),
    "critic": ObservationGroupCfg(
      terms=dict(actor_terms),
      concatenate_terms=True,
      enable_corruption=False,
    ),
  }

  ##
  # Actions: legs only, flat scale (D4, lockstep with PolicyBackend.cpp).
  ##

  actions: dict[str, ActionTermCfg] = {
    "joint_pos": JointPositionActionCfg(
      entity_name="robot",
      actuator_names=LEG_JOINT_NAMES,
      scale=0.5,
      use_default_offset=True,
    )
  }

  ##
  # Commands (D5).
  ##

  commands: dict[str, CommandTermCfg] = {
    "twist": UniformVelocityCommandCfg(
      entity_name="robot",
      resampling_time_range=(3.0, 8.0),
      rel_standing_envs=0.1,
      heading_command=False,
      debug_vis=True,
      ranges=UniformVelocityCommandCfg.Ranges(
        lin_vel_x=(-0.3, 0.6),
        lin_vel_y=(-0.2, 0.2),
        ang_vel_z=(-0.6, 0.6),
      ),
    )
  }

  ##
  # Events: minimal resets + push perturbation. No terrain/geom-name-dependent DR
  # (foot_friction/base_com) since K1_22dof.xml has no named "_collision" geoms or
  # feet-only collision groups the way G1's XML does (v1 simplification, flagged).
  ##

  events = {
    "reset_base": EventTermCfg(
      func=envs_mdp.reset_root_state_uniform,
      mode="reset",
      params={
        "pose_range": {
          "x": (-0.5, 0.5), "y": (-0.5, 0.5), "z": (0.0, 0.02), "yaw": (-3.14, 3.14),
        },
        "velocity_range": {},
      },
    ),
    "reset_robot_joints": EventTermCfg(
      func=envs_mdp.reset_joints_by_offset,
      mode="reset",
      params={
        "position_range": (0.0, 0.0),
        "velocity_range": (0.0, 0.0),
        "asset_cfg": SceneEntityCfg("robot", joint_names=(".*",)),
      },
    ),
    "push_robot": EventTermCfg(
      func=envs_mdp.push_by_setting_velocity,
      mode="interval",
      interval_range_s=(2.0, 5.0),
      params={
        "velocity_range": {
          "x": (-0.3, 0.3), "y": (-0.3, 0.3), "yaw": (-0.5, 0.5),
        },
      },
    ),
  }

  ##
  # Rewards: ParameterWalk math, reduced set (D2). Canned mjlab terms reused
  # where they map 1:1; the rest are custom (tasks/k1_walk/mdp.py).
  ##

  # actuator_force/qfrc_actuator-based terms need different SceneEntityCfg key
  # fields: joint_torques_l2/torque_tiredness read asset.data.actuator_force
  # (indexed by asset_cfg.actuator_ids -> needs actuator_names=), while
  # electrical_power_cost reads asset.data.qfrc_actuator (joint-space, indexed by
  # asset_cfg.joint_ids -> needs joint_names=).
  leg_actuator_cfg = SceneEntityCfg("robot", actuator_names=LEG_JOINT_NAMES)
  leg_joint_cfg = SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES)
  all_joints_cfg = SceneEntityCfg("robot", joint_names=(".*",))

  rewards = {
    "survival": RewardTermCfg(func=envs_mdp.is_alive, weight=0.25),
    "track_lin_vel": RewardTermCfg(
      func=velocity_mdp.track_linear_velocity,
      weight=2.0,
      params={"command_name": "twist", "std": math.sqrt(TRACKING_SIGMA)},
    ),
    "track_ang_vel": RewardTermCfg(
      func=velocity_mdp.track_angular_velocity,
      weight=1.5,
      params={"command_name": "twist", "std": math.sqrt(TRACKING_SIGMA)},
    ),
    "base_height": RewardTermCfg(
      func=k1_mdp.base_height,
      weight=-20.0,
      params={"target_height": BASE_HEIGHT_TARGET},
    ),
    "orientation": RewardTermCfg(func=envs_mdp.flat_orientation_l2, weight=-20.0),
    "torques": RewardTermCfg(
      func=envs_mdp.joint_torques_l2,
      weight=-3e-4,
      params={"asset_cfg": leg_actuator_cfg},
    ),
    "torque_tiredness": RewardTermCfg(
      func=k1_mdp.torque_tiredness,
      weight=-1e-2,
      params={"asset_cfg": leg_actuator_cfg, "torque_limit": LEG_TORQUE_LIMIT},
    ),
    "power": RewardTermCfg(
      func=envs_mdp.electrical_power_cost,
      weight=-3e-3,
      params={"asset_cfg": leg_joint_cfg},
    ),
    "lin_vel_z": RewardTermCfg(func=k1_mdp.lin_vel_z, weight=-2.0),
    "ang_vel_xy": RewardTermCfg(func=k1_mdp.ang_vel_xy, weight=-0.2),
    "dof_vel": RewardTermCfg(
      func=envs_mdp.joint_vel_l2, weight=-2e-4, params={"asset_cfg": all_joints_cfg}
    ),
    "dof_acc": RewardTermCfg(
      func=envs_mdp.joint_acc_l2, weight=-2e-7, params={"asset_cfg": all_joints_cfg}
    ),
    "action_rate": RewardTermCfg(func=envs_mdp.action_rate_l2, weight=-1.5),
    "dof_pos_limits": RewardTermCfg(
      func=envs_mdp.joint_pos_limits, weight=-1.0, params={"asset_cfg": all_joints_cfg}
    ),
    "collision": RewardTermCfg(
      func=k1_mdp.collision,
      weight=-1.0,
      params={"sensor_name": SELF_COLLISION_SENSOR, "force_threshold": 1.0},
    ),
    "feet_slip": RewardTermCfg(
      func=velocity_mdp.feet_slip,
      weight=-0.1,
      params={
        "sensor_name": FEET_CONTACT_SENSOR,
        "command_name": "twist",
        "command_threshold": 0.05,
        "asset_cfg": SceneEntityCfg("robot", site_names=("left_foot", "right_foot")),
      },
    ),
    "feet_swing": RewardTermCfg(
      func=k1_mdp.feet_swing,
      weight=3.0,
      params={"sensor_name": FEET_CONTACT_SENSOR, "gait_frequency": GAIT_FREQUENCY},
    ),
  }

  ##
  # Terminations.
  ##

  terminations = {
    "time_out": TerminationTermCfg(func=envs_mdp.time_out, time_out=True),
    "bad_orientation": TerminationTermCfg(
      func=envs_mdp.bad_orientation, params={"limit_angle": math.radians(70.0)}
    ),
    "fell": TerminationTermCfg(
      func=envs_mdp.root_height_below_minimum,
      params={"minimum_height": TERMINATE_HEIGHT},
    ),
  }

  ##
  # Scene: flat plane, no terrain scan (D1 obs has no height_scan term).
  ##

  feet_ground_contact = ContactSensorCfg(
    name=FEET_CONTACT_SENSOR,
    primary=ContactMatch(
      mode="body", pattern=r"^(left|right)_foot_link$", entity="robot"
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
  )
  self_collision = ContactSensorCfg(
    name=SELF_COLLISION_SENSOR,
    primary=ContactMatch(mode="subtree", pattern="Trunk", entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern="Trunk", entity="robot"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )

  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(terrain_type="plane"),
      entities={"robot": get_k1_robot_cfg()},
      sensors=(feet_ground_contact, self_collision),
      num_envs=1,
      extent=2.0,
    ),
    observations=observations,
    actions=actions,
    commands=commands,
    events=events,
    rewards=rewards,
    terminations=terminations,
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY,
      entity_name="robot",
      body_name="Trunk",
      distance=2.0,
      elevation=-10.0,
      azimuth=90.0,
    ),
    sim=SimulationCfg(
      mujoco=MujocoCfg(timestep=0.001, iterations=10, ls_iterations=20),
    ),
    decimation=20,  # 1kHz physics / 20 = 50Hz control (D6, matches inference_divisor).
    episode_length_s=20.0,
  )

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)

  return cfg
