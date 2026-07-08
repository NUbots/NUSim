"""Policy base class: the MJX environment + the overridable cost hooks.

A concrete policy (e.g. `policies/walk.py`) subclasses `Policy` and overrides
`reward()` (the cost method), `sample_command()`, and optionally `termination()` /
`domain_randomize()`. Everything else — loading the K1 MuJoCo model into MJX,
building the frozen 47-dim observation, applying the 12-dim leg action with the
gait clock, and the PD sub-loop — lives here so subclasses stay small and every
policy shares one deploy contract (training/OBS_ACTION_CONTRACT.md).

The class implements the brax environment interface (`reset`/`step` returning a
`brax.envs.base.State`), so `train.py` can hand it to brax's PPO directly.
"""
from __future__ import annotations

import os

import jax
import jax.numpy as jp
import mujoco
import yaml
from brax.envs.base import Env, State
from mujoco import mjx

# JointIndexK1: the 12 leg joints are the contiguous slice [10, 22).
LEG_START = 10
LEG_COUNT = 12
NUM_JOINTS = 22
OBS_DIM = 47
ACTION_DIM = 12

_HERE = os.path.dirname(os.path.abspath(__file__))
_MUJOCO_DIR = os.path.dirname(_HERE)


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


class Policy(Env):
    """Base MJX locomotion environment. Subclass and override the cost hooks."""

    # --- configuration knobs (override in subclass __init__ or config) ---
    scene_xml = "models/k1/k1_scene_flat.xml"   # relative to mujoco/
    physics_steps_per_control = 20               # 1 kHz physics / 20 = 50 Hz control
    action_scale = 0.5                           # MUST match config/locomotion.yaml policy.action_scale
    gait_frequency = 1.5                         # Hz
    stand_threshold = 0.05
    episode_length = 1000                        # control steps
    terminate_height = 0.35                      # base z below this ⇒ fall
    base_height_target = 0.52

    def __init__(self, config: dict | None = None):
        cfg = config or {}
        for k, v in cfg.get("policy", {}).items():
            setattr(self, k, v)

        # Load the K1 model into MuJoCo then MJX.
        xml_path = os.path.join(_MUJOCO_DIR, self.scene_xml)
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_model.opt.timestep = 0.001
        # MJX supports a limited set of collision pairs (no cylinder↔mesh, etc.).
        # For locomotion training we only need foot↔floor contact, so prune every
        # other collider — this mutates the loaded model in Python only; the C++
        # deploy sim keeps the full-collision model untouched.
        self._prune_collisions_for_mjx()
        self.mjx_model = mjx.put_model(self.mj_model)
        self.dt = self.mj_model.opt.timestep * self.physics_steps_per_control

        # PD gains + ready pose (config/gains.yaml, 22-vector JointIndexK1 order).
        gains = _load_yaml(os.path.join(_MUJOCO_DIR, "config", "gains.yaml"))
        self.kp = jp.array(gains["kp"], dtype=jp.float32)
        self.kd = jp.array(gains["kd"], dtype=jp.float32)
        self.default_pose = jp.array(gains["ready_pose"], dtype=jp.float32)

        # Index maps: actuator/joint order in the model vs JointIndexK1.
        self._resolve_indices()

        # `ready` keyframe qpos for resets (falls back to model qpos0).
        key = self.mj_model.keyframe("ready") if self._has_keyframe("ready") else None
        self.init_qpos = jp.array(key.qpos if key is not None else self.mj_model.qpos0, dtype=jp.float32)

    # ------------------------------------------------------------------ setup
    def _prune_collisions_for_mjx(self):
        """Keep only floor↔foot collisions; disable all other colliders (MJX-safe).

        Foot geoms are the box/capsule/sphere colliders on bodies whose name
        contains 'foot' or 'ankle'; the floor is the ground plane. Everything else
        becomes visual-only (contype=conaffinity=0)."""
        m = self.mj_model
        prim = {int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_CAPSULE), int(mujoco.mjtGeom.mjGEOM_SPHERE)}
        kept = 0
        for g in range(m.ngeom):
            gname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            bname = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "").lower()
            is_floor = m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE or gname == "floor"
            is_foot = ("foot" in bname or "ankle" in bname) and int(m.geom_type[g]) in prim
            if is_floor or is_foot:
                m.geom_contype[g] = 1
                m.geom_conaffinity[g] = 1
                kept += 1 if not is_floor else 0
            else:
                m.geom_contype[g] = 0
                m.geom_conaffinity[g] = 0
        if kept == 0:
            raise RuntimeError("no foot collision geoms found for MJX training scene")

    def _has_keyframe(self, name):
        try:
            self.mj_model.keyframe(name)
            return True
        except Exception:
            return False

    def _resolve_indices(self):
        from shared_names import JOINT_NAMES  # local helper module (names in JointIndexK1 order)

        self.qpos_adr = []
        self.dof_adr = []
        self.act_adr = []
        for name in JOINT_NAMES:
            jid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, name)
            aid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            self.qpos_adr.append(self.mj_model.jnt_qposadr[jid])
            self.dof_adr.append(self.mj_model.jnt_dofadr[jid])
            self.act_adr.append(aid)
        self.qpos_adr = jp.array(self.qpos_adr)
        self.dof_adr = jp.array(self.dof_adr)
        self.act_adr = jp.array(self.act_adr)
        # root free joint = first 7 qpos / 6 qvel (trunk).
        self.root_qadr = 0
        self.root_vadr = 0
        # gyro sensor address (angular-velocity) if present.
        sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, "angular-velocity")
        self.gyro_adr = int(self.mj_model.sensor_adr[sid]) if sid >= 0 else -1

    # --------------------------------------------------------------- brax API
    @property
    def observation_size(self):
        return OBS_DIM

    @property
    def action_size(self):
        return ACTION_DIM

    @property
    def backend(self):
        return "mjx"

    def reset(self, rng: jax.Array) -> State:
        rng, c_rng, d_rng = jax.random.split(rng, 3)
        data = mjx.make_data(self.mjx_model).replace(qpos=self.init_qpos)
        data = self.domain_randomize(data, d_rng)
        data = mjx.forward(self.mjx_model, data)
        command = self.sample_command(c_rng)
        info = {
            "rng": rng,
            "command": command,
            "gait_phase": jp.float32(0.0),
            "last_action": jp.zeros(ACTION_DIM),
            "step": jp.int32(0),
        }
        obs = self._observation(data, info)
        metrics = {"reward": jp.float32(0.0), "x_vel": jp.float32(0.0)}
        return State(data, obs, jp.float32(0.0), jp.float32(0.0), metrics, info)

    def step(self, state: State, action: jax.Array) -> State:
        info = dict(state.info)
        moving = jp.linalg.norm(info["command"]) > self.stand_threshold
        gait_phase = jp.where(
            moving, jp.mod(info["gait_phase"] + self.dt * self.gait_frequency, 1.0), 0.0
        )

        # Target = ready pose; overwrite the 12 legs with the scaled action.
        q_ref = self.default_pose
        leg_targets = self.default_pose[LEG_START:LEG_START + LEG_COUNT] + self.action_scale * action
        q_ref = q_ref.at[LEG_START:LEG_START + LEG_COUNT].set(leg_targets)

        data = self._pd_rollout(state.pipeline_state, q_ref)

        info["gait_phase"] = gait_phase
        info["last_action"] = action
        info["step"] = info["step"] + 1
        obs = self._observation(data, info)

        reward = self.reward(data, action, info)
        done = jp.where(self.termination(data, info), 1.0, 0.0)

        metrics = dict(state.metrics)
        metrics["reward"] = reward
        metrics["x_vel"] = self._base_lin_vel(data)[0]
        return state.replace(pipeline_state=data, obs=obs, reward=reward, done=done, metrics=metrics, info=info)

    # ------------------------------------------------------------- mechanics
    def _pd_rollout(self, data, q_ref):
        """Run the PD-tracked physics sub-loop (physics_steps_per_control steps)."""

        def pd_step(d, _):
            q = d.qpos[self.qpos_adr]
            dq = d.qvel[self.dof_adr]
            tau = self.kp * (q_ref - q) - self.kd * dq
            ctrl = jp.zeros(self.mjx_model.nu).at[self.act_adr].set(tau)
            d = d.replace(ctrl=ctrl)
            return mjx.step(self.mjx_model, d), None

        data, _ = jax.lax.scan(pd_step, data, None, length=self.physics_steps_per_control)
        return data

    def _base_quat(self, data):
        return data.qpos[3:7]  # wxyz

    def _base_lin_vel(self, data):
        # world-frame linear velocity rotated into the body frame.
        return _rotate_inv(self._base_quat(data), data.qvel[0:3])

    def _base_ang_vel(self, data):
        if self.gyro_adr >= 0:
            return data.sensordata[self.gyro_adr:self.gyro_adr + 3]
        return _rotate_inv(self._base_quat(data), data.qvel[3:6])

    def _projected_gravity(self, data):
        return _rotate_inv(self._base_quat(data), jp.array([0.0, 0.0, -1.0]))

    def _observation(self, data, info):
        moving = jp.linalg.norm(info["command"]) > self.stand_threshold
        cmd = jp.where(moving, info["command"], 0.0)
        phase = info["gait_phase"]
        gait = jp.where(moving, jp.array([jp.cos(2 * jp.pi * phase), jp.sin(2 * jp.pi * phase)]), jp.zeros(2))
        q = data.qpos[self.qpos_adr]
        dq = data.qvel[self.dof_adr]
        leg = slice(LEG_START, LEG_START + LEG_COUNT)
        return jp.concatenate([
            self._projected_gravity(data),                 # 3
            self._base_ang_vel(data),                      # 3
            cmd,                                           # 3
            gait,                                          # 2
            (q[leg] - self.default_pose[leg]),             # 12
            dq[leg] * 0.1,                                 # 12
            info["last_action"],                           # 12
        ])

    # --------------------------------------------------------- OVERRIDE HOOKS
    def sample_command(self, rng: jax.Array) -> jax.Array:
        """Return a [vx, vy, vyaw] command. Override to set ranges."""
        return jp.zeros(3)

    def reward(self, data, action, info) -> jax.Array:
        """THE cost method — return a scalar reward. Override in the subclass."""
        raise NotImplementedError("Policy subclasses must implement reward()")

    def termination(self, data, info) -> jax.Array:
        """Episode-ending condition (default: base fell below terminate_height)."""
        return data.qpos[2] < self.terminate_height

    def domain_randomize(self, data, rng: jax.Array):
        """Optional per-reset randomization of the initial state. Default: none."""
        return data


def _rotate_inv(quat_wxyz, vec):
    """Rotate `vec` by the inverse of a wxyz quaternion (world→body)."""
    w, x, y, z = quat_wxyz
    # inverse = conjugate for a unit quaternion
    q = jp.array([w, -x, -y, -z])
    return _quat_rotate(q, vec)


def _quat_rotate(q, v):
    w, x, y, z = q
    uv = jp.array([
        y * v[2] - z * v[1],
        z * v[0] - x * v[2],
        x * v[1] - y * v[0],
    ])
    uuv = jp.array([
        y * uv[2] - z * uv[1],
        z * uv[0] - x * uv[2],
        x * uv[1] - y * uv[0],
    ])
    return v + 2 * (w * uv + uuv)
