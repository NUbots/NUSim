"""Custom mdp terms for the K1 walk task.

These exist because mjlab's canned `mjlab.envs.mdp` / `mjlab.tasks.velocity.mdp`
terms don't cover gait-clock-gated command observation or the reduced
ParameterWalk reward set (PORT_DECISIONS.md D2). Canned terms are reused directly
in `env_cfg.py` wherever they map 1:1 (track_linear_velocity, track_angular_velocity,
flat_orientation_l2, joint_torques_l2, joint_vel_l2, joint_acc_l2, action_rate_l2,
joint_pos_limits, electrical_power_cost, is_alive, feet_slip, root_height_below_minimum)
and are not reimplemented here.

Gait phase note: the phase buffer (`env.k1_gait_phase`) is advanced exactly once per
env step, inside `k1_command_gait_clock.__call__` (an observation term). Per
`ManagerBasedRlEnv.step()`'s pipeline (mjlab/envs/manager_based_rl_env.py), rewards
are computed *before* observations each step, so `feet_swing`'s read of
`env.k1_gait_phase` sees the phase that was current when the action being rewarded
was produced (i.e. it reads before this step's advance) -- exactly matching
PolicyBackend.cpp's semantics where `gait_phase_` is advanced once per inference and
consumed by the same inference's obs. Do not add a second phase-advancing term.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import torch

from mjlab.entity import Entity
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactSensor

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

_DEFAULT_ASSET_CFG = SceneEntityCfg("robot")


def _gait_phase_buf(env: ManagerBasedRlEnv) -> torch.Tensor:
  """Lazily create/return the per-env gait phase buffer, attached to `env`."""
  buf = getattr(env, "k1_gait_phase", None)
  if buf is None:
    buf = torch.zeros(env.num_envs, device=env.device)
    env.k1_gait_phase = buf
  return buf


class k1_command_gait_clock:
  """[vx, vy, vyaw, cos(2*pi*phase), sin(2*pi*phase)], gated + advanced.

  Reproduces PolicyBackend.cpp::build_obs offsets [6:11] bit-for-bit (D1/D5):
  command and gait-clock terms are zeroed while `||cmd|| <= stand_threshold`;
  phase advances by `step_dt * gait_frequency` (mod 1) while moving and resets to
  0 while standing (PolicyBackend.cpp:143-147). `gait_frequency` is a fixed
  constant here (not a sampled command dim), matching D5.
  """

  def __init__(self, cfg, env: ManagerBasedRlEnv):
    del cfg
    self._env = env
    _gait_phase_buf(env)  # Ensure the buffer exists before first use.

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    command_name: str,
    stand_threshold: float = 0.05,
    gait_frequency: float = 1.5,
  ) -> torch.Tensor:
    command = env.command_manager.get_command(command_name)
    assert command is not None
    speed = torch.linalg.norm(command, dim=1)
    moving = speed > stand_threshold

    phase = _gait_phase_buf(env)
    phase[:] = torch.where(
      moving,
      torch.fmod(phase + env.step_dt * gait_frequency, 1.0),
      torch.zeros_like(phase),
    )

    moving_f = moving.float().unsqueeze(-1)
    cos_sin = torch.stack(
      (torch.cos(2.0 * math.pi * phase), torch.sin(2.0 * math.pi * phase)), dim=-1
    )
    return torch.cat((command * moving_f, cos_sin * moving_f), dim=-1)

  def reset(self, env_ids: torch.Tensor) -> None:
    _gait_phase_buf(self._env)[env_ids] = 0.0


##
# Rewards: ParameterWalk formulas (PORT_SPEC ss1b), reduced set (D2).
# Terms with a direct canned-mdp equivalent are NOT reimplemented here (see module
# docstring); only the ones with no mjlab equivalent live in this file.
##


def base_height(
  env: ManagerBasedRlEnv,
  target_height: float,
  asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG,
) -> torch.Tensor:
  """(base_z - target_height)^2. Flat terrain, so terrain_z == 0 (PORT_SPEC ss1b)."""
  asset: Entity = env.scene[asset_cfg.name]
  base_z = asset.data.root_link_pos_w[:, 2]
  return torch.square(base_z - target_height)


class torque_tiredness:
  """sum(clip((torque / torque_limit)^2, max=1)) over the given actuators.

  Note: indexed by *actuator* (asset_cfg.actuator_ids / actuator_force), not joint
  -- ``asset_cfg`` must be built with ``actuator_names=`` (matches
  ``envs.mdp.rewards.joint_torques_l2``'s indexing convention). Our K1 actuators are
  named identically to their joints, so the ``torque_limit`` dict is still keyed by
  joint/actuator name interchangeably.
  """

  def __init__(self, cfg: RewardTermCfg, env: ManagerBasedRlEnv):
    asset: Entity = env.scene[cfg.params["asset_cfg"].name]
    actuator_names = cfg.params["asset_cfg"].actuator_names
    assert actuator_names is not None, "torque_tiredness requires actuator_names"
    _, matched_names = asset.find_actuators(actuator_names)
    torque_limit = cfg.params["torque_limit"]
    limits = [torque_limit[n] for n in matched_names]
    self.torque_limit = torch.tensor(limits, device=env.device, dtype=torch.float32)

  def __call__(
    self,
    env: ManagerBasedRlEnv,
    torque_limit: dict[str, float],
    asset_cfg: SceneEntityCfg,
  ) -> torch.Tensor:
    del torque_limit
    asset: Entity = env.scene[asset_cfg.name]
    torque = asset.data.actuator_force[:, asset_cfg.actuator_ids]
    ratio_sq = torch.square(torque / self.torque_limit)
    return torch.sum(torch.clamp(ratio_sq, max=1.0), dim=1)


def lin_vel_z(
  env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
  """base_lin_vel_z^2 (body frame)."""
  asset: Entity = env.scene[asset_cfg.name]
  return torch.square(asset.data.root_link_lin_vel_b[:, 2])


def ang_vel_xy(
  env: ManagerBasedRlEnv, asset_cfg: SceneEntityCfg = _DEFAULT_ASSET_CFG
) -> torch.Tensor:
  """sum(base_ang_vel_xy^2) (body frame gyro-equivalent, matches obs term)."""
  asset: Entity = env.scene[asset_cfg.name]
  return torch.sum(torch.square(asset.data.root_link_ang_vel_b[:, :2]), dim=1)


def collision(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  force_threshold: float = 1.0,
) -> torch.Tensor:
  """Count of self-contacts on the watched bodies with force > force_threshold.

  ParameterWalk's `collision` reward (PORT_SPEC ss1b) counts contacts on
  `penalize_contacts_on` bodies (Trunk/H1/H2/AL/AR/Waist/Hip/Shank/Ankle) with
  force > 1N. Wired to a `ContactSensorCfg` (self-collision, primary==secondary)
  in env_cfg.py.
  """
  sensor: ContactSensor = env.scene[sensor_name]
  data = sensor.data
  if data.force_history is not None:
    force_mag = torch.norm(data.force_history, dim=-1)  # [B, N, H]
    return (force_mag > force_threshold).any(dim=-1).sum(dim=-1).float()
  assert data.found is not None
  return data.found.sum(dim=-1).float()


def feet_swing(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  gait_frequency: float = 1.5,
  swing_period: float = 0.5,
) -> torch.Tensor:
  """Reward swing-leg-off-ground at gait phase 0.25/0.75 +/- swing_period/2.

  ParameterWalk's `feet_swing` (PORT_SPEC ss1b, weight +3.0): rewards each foot being
  airborne during its expected swing window (left swings around phase 0.25,
  right around phase 0.75, symmetric walk). Reads the same `env.k1_gait_phase`
  buffer the observation term advances (see module docstring for ordering).
  """
  sensor: ContactSensor = env.scene[sensor_name]
  assert sensor.data.found is not None
  in_air = (sensor.data.found == 0).float()  # [B, 2] == [left, right]

  phase = _gait_phase_buf(env)
  half = swing_period / 2.0

  def _in_window(center: float) -> torch.Tensor:
    d = torch.remainder(phase - center + 0.5, 1.0) - 0.5
    return (d.abs() < half).float()

  left_expected = _in_window(0.25)
  right_expected = _in_window(0.75)
  expected = torch.stack((left_expected, right_expected), dim=-1)
  return torch.sum(expected * in_air, dim=1)
