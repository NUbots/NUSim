"""Walk: a velocity-tracking locomotion policy for the K1.

`./b train walk.py` trains this. It overrides the cost (`reward`) and the command
sampler; everything else (obs, action, gait clock, PD) comes from `Policy`. Reward
terms are ported from htwk-gym's K1 ParameterWalk (velocity tracking + upright +
height + regularization), which the user asked to use as the reference.

To make your own policy: copy this file, tweak `reward()` and `sample_command()`,
and `./b train yourpolicy.py`.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import jax
import jax.numpy as jp

from policy import Policy


class Walk(Policy):
    # Command ranges (m/s, m/s, rad/s), sampled per episode.
    vx_range = (-0.3, 0.6)
    vy_range = (-0.2, 0.2)
    vyaw_range = (-0.6, 0.6)

    # Reward weights (per control step; tune these). Modeled on htwk ParameterWalk.
    w_track_lin = 1.5      # exp velocity tracking (x,y)
    w_track_ang = 0.8      # exp yaw-rate tracking
    w_upright = 0.5        # penalize non-upright
    w_height = 0.3         # penalize base-height error
    w_lin_vel_z = 0.3      # penalize vertical bob
    w_ang_vel_xy = 0.1     # penalize roll/pitch rate
    w_action_rate = 0.02   # penalize action jitter
    w_torque = 0.0002      # penalize effort (proxy: action magnitude)
    w_alive = 0.5          # survival bonus
    track_sigma = 0.25     # tracking sharpness

    def sample_command(self, rng: jax.Array) -> jax.Array:
        k1, k2, k3 = jax.random.split(rng, 3)
        vx = jax.random.uniform(k1, (), minval=self.vx_range[0], maxval=self.vx_range[1])
        vy = jax.random.uniform(k2, (), minval=self.vy_range[0], maxval=self.vy_range[1])
        vyaw = jax.random.uniform(k3, (), minval=self.vyaw_range[0], maxval=self.vyaw_range[1])
        return jp.array([vx, vy, vyaw])

    def reward(self, data, action, info) -> jax.Array:
        cmd = info["command"]
        lin = self._base_lin_vel(data)      # body frame
        ang = self._base_ang_vel(data)

        track_lin = jp.exp(-jp.sum((cmd[0:2] - lin[0:2]) ** 2) / self.track_sigma)
        track_ang = jp.exp(-((cmd[2] - ang[2]) ** 2) / self.track_sigma)
        upright = -jp.sum(self._projected_gravity(data)[0:2] ** 2)   # 0 when upright
        height = -((data.qpos[2] - self.base_height_target) ** 2)
        lin_vel_z = -(lin[2] ** 2)
        ang_vel_xy = -jp.sum(ang[0:2] ** 2)
        action_rate = -jp.sum((action - info["last_action"]) ** 2)
        torque = -jp.sum(action ** 2)
        alive = 1.0

        r = (
            self.w_track_lin * track_lin
            + self.w_track_ang * track_ang
            + self.w_upright * upright
            + self.w_height * height
            + self.w_lin_vel_z * lin_vel_z
            + self.w_ang_vel_xy * ang_vel_xy
            + self.w_action_rate * action_rate
            + self.w_torque * torque
            + self.w_alive * alive
        )
        return r * self.dt


# `./b train` / train.py look for a module-level `POLICY` class.
POLICY = Walk
