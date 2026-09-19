#!/usr/bin/env python3
"""Run the deployed K1 walk policy against the NUSim model across ground surfaces.

This reproduces module::skill::K1WalkPolicy's control path -- the same observation
assembly, the same 50 Hz tick, the same tau = kp*(q_ref - q) - kd*dq law with the gains
from K1WalkPolicy.yaml -- directly on top of the NUSim MJCF, with no NUClear, DDS or
docker in the loop. The point is to be able to change one variable (the ground) and
measure what the robot's feet do.

Why this exists: on hardware the same policy at the same commanded velocity is stable on
carpet and progressively loses balance on the synthetic-grass field, with tip-toe
appearing during the degradation. Everything identical across those two runs cannot be the
cause, which leaves foot-ground interaction. NUSim could not show that difference before,
for two reasons this script controls explicitly:

  * The K1 foot box geom declares no friction, so it takes MuJoCo's default of 1.0, and
    with equal geom priorities MuJoCo uses the element-wise MAX of the two geoms -- so the
    robocup scene's "0.8 grass" floor has always simulated mu = 1.0. --friction raises the
    floor's priority so its number actually governs.
  * The foot's solref timeconst (0.001 s, one model timestep) makes contact effectively
    rigid. --solref-timeconst sets the floor's, which then wins on priority.

Tip-toe is reported as the centre of pressure in the foot's own frame. The sole box spans
x in [-0.064, +0.116], so a flat-footed stance sits near cop_x = 0.026 and a foot rolled
onto its toe drives cop_x towards +0.116.

Usage:
    python3 tools/walk_surface_sweep.py \
        --policy ~/NUbots_K1/module/skill/K1WalkPolicy/data/k1_walk_v1_20260723_617M.onnx \
        --config ~/NUbots_K1/module/skill/K1WalkPolicy/data/config/K1WalkPolicy.yaml \
        --vx 0.2 --friction 1.0 0.6 0.4 0.3 0.25

Both the 82-observation (with base linear velocity) and 79-observation (without) contracts
are supported; which one is used is read off the ONNX input width.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys

import numpy as np
import onnxruntime as ort
import yaml

import mujoco

# JointIndexK1 serial order, matching the actuator order in models/k1/K1_22dof.xml and the
# joint arrays in K1WalkPolicy.yaml.
JOINT_NAMES = [
    "AAHead_yaw",
    "Head_pitch",
    "ALeft_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "ARight_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
]
HEAD_SLOTS = (0, 1)
LEFT_KNEE, RIGHT_KNEE = 13, 19
NJOINT = len(JOINT_NAMES)


class Robot:
    """Index maps and the deployment-side control law for one K1 in a loaded model."""

    def __init__(self, model: mujoco.MjModel):
        self.m = model
        self.qpos_adr = np.array(
            [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES]
        )
        self.dof_adr = np.array(
            [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES]
        )
        self.act_id = np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINT_NAMES])
        if np.any(self.act_id < 0) or np.any(self.qpos_adr < 0):
            raise SystemExit("model is missing one of the 22 K1 joints/actuators")

        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        if root < 0:
            raise SystemExit("model has no free joint named 'root'")
        self.root_qpos = model.jnt_qposadr[root]
        self.root_dof = model.jnt_dofadr[root]

        self.left_foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_foot_link")
        self.right_foot = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_foot_link")

        self.sens_quat = self._sensor_adr("orientation")
        self.sens_gyro = self._sensor_adr("angular-velocity")

    def _sensor_adr(self, name: str) -> int:
        sid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return int(self.m.sensor_adr[sid]) if sid >= 0 else -1

    def q(self, d):
        return d.qpos[self.qpos_adr]

    def dq(self, d):
        return d.qvel[self.dof_adr]

    def base_rot(self, d) -> np.ndarray:
        """World-from-torso rotation, from the IMU quaternion sensor."""
        if self.sens_quat >= 0:
            quat = d.sensordata[self.sens_quat : self.sens_quat + 4].copy()
        else:
            quat = d.xquat[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "Trunk")].copy()
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        return R.reshape(3, 3)

    def gyro(self, d) -> np.ndarray:
        if self.sens_gyro >= 0:
            return d.sensordata[self.sens_gyro : self.sens_gyro + 3].copy()
        return np.zeros(3)

    def apply_pd(self, d, q_ref: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> None:
        """K1WalkPolicy streams q with dq = 0 and tau = 0; the platform closes the loop."""
        tau = kp * (q_ref - self.q(d)) - kd * self.dq(d)
        lo = self.m.actuator_forcerange[self.act_id, 0]
        hi = self.m.actuator_forcerange[self.act_id, 1]
        limited = self.m.actuator_forcelimited[self.act_id].astype(bool)
        tau = np.where(limited, np.clip(tau, lo, hi), tau)
        d.ctrl[self.act_id] = tau

    def foot_contacts(self, d):
        """(fz, cop_x, sole_pitch) for the left and right foot."""
        out = []
        for body in (self.left_foot, self.right_foot):
            R = d.xmat[body].reshape(3, 3)
            org = d.xpos[body]
            pitch = math.atan2(R[0, 2], R[2, 2])
            fz, cop = 0.0, 0.0
            for c in range(d.ncon):
                con = d.contact[c]
                if body not in (self.m.geom_bodyid[con.geom1], self.m.geom_bodyid[con.geom2]):
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.m, d, c, force)
                fn = float(force[0])
                if fn <= 0.0:
                    continue
                local = R.T @ (con.pos - org)
                fz += fn
                cop += fn * local[0]
            out.append((fz, cop / fz if fz > 0 else float("nan"), pitch))
        return out


def load_policy_config(path: pathlib.Path) -> dict:
    cfg = yaml.safe_load(path.read_text())
    out = {
        "kp": np.array(cfg["kp"], dtype=float),
        "kd": np.array(cfg["kd"], dtype=float),
        "default_pose": np.array(cfg["default_pose"], dtype=float),
        "action_scale": np.array(cfg["action_scale_joint"], dtype=float) * float(cfg.get("action_scale", 1.0)),
        "gait_frequency": float(cfg["gait_frequency"]),
        "stand_threshold": float(cfg["stand_threshold"]),
        "linvel_alpha": float(cfg.get("linvel_alpha", 0.3)),
        "head_kp": float(cfg["head"]["kp"]),
        "head_kd": float(cfg["head"]["kd"]),
    }
    lower, upper = cfg.get("joint_lower"), cfg.get("joint_upper")
    out["joint_lower"] = np.array(lower, dtype=float) if lower else None
    out["joint_upper"] = np.array(upper, dtype=float) if upper else None
    return out


def apply_surface(model: mujoco.MjModel, friction: float, timeconst: float, dampratio: float) -> None:
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if floor < 0:
        raise SystemExit("scene has no geom named 'floor'")
    timeconst = max(timeconst, 2.0 * model.opt.timestep)
    model.geom_priority[floor] = 1
    model.geom_friction[floor, 0] = friction
    model.geom_solref[floor] = [timeconst, dampratio]


class Corruption:
    """The ways the deployment path differs from an exact simulator.

    NUSim publishes model state directly: exact, instantaneous, never stale. A deployment
    path is defined by what the robot gets wrong, so each of these is a knob rather than an
    assumption, and every one of them is something the hardware plausibly does.
    """

    def __init__(self, args):
        self.linvel_mode = args.linvel_mode
        self.odom_rate = args.odom_rate
        self.obs_delay = args.obs_delay
        self.action_delay = args.action_delay
        self.imu_noise = args.imu_noise

    def describe(self) -> str:
        return (
            f"linvel={self.linvel_mode} odom={self.odom_rate}Hz obs_delay={self.obs_delay} "
            f"act_delay={self.action_delay} imu_noise={self.imu_noise}"
        )


def delay_weights(delay: float, taps: int) -> np.ndarray:
    """Linear-interpolation weights over a history buffer, index 0 = freshest."""
    idx = np.arange(taps, dtype=float)
    return np.clip(1.0 - np.abs(idx - delay), 0.0, 1.0)


def run_one(
    scene: pathlib.Path,
    policy: ort.InferenceSession,
    obs_dim: int,
    cfg: dict,
    corrupt: "Corruption",
    friction: float,
    timeconst: float,
    dampratio: float,
    vx: float,
    settle_s: float,
    walk_s: float,
    keyframe: str,
    seed: int,
    trace_path: pathlib.Path | None,
) -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene))
    apply_surface(model, friction, timeconst, dampratio)
    data = mujoco.MjData(model)

    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
    if key < 0:
        raise SystemExit(f"scene has no keyframe '{keyframe}'")
    mujoco.mj_resetDataKeyframe(model, data, key)
    mujoco.mj_forward(model, data)

    robot = Robot(model)
    ctrl_dt = 0.02
    substeps = max(1, int(round(ctrl_dt / model.opt.timestep)))

    kp = cfg["kp"].copy()
    kd = cfg["kd"].copy()
    kp[list(HEAD_SLOTS)] = cfg["head_kp"]
    kd[list(HEAD_SLOTS)] = cfg["head_kd"]

    last_action = np.zeros(NJOINT, dtype=np.float32)
    phase = np.array([0.0, math.pi])
    linvel_body = np.zeros(2)
    input_name = policy.get_inputs()[0].name
    rng = np.random.default_rng(seed)

    obs_taps = int(math.ceil(corrupt.obs_delay)) + 1
    obs_hist = None
    obs_w = delay_weights(corrupt.obs_delay, obs_taps)
    prev_q_ref = cfg["default_pose"].copy()
    # Odometry arrival cadence for the aliased mode: K1WalkPolicy differentiates whatever
    # rt/odometer_state last said, so an odometry channel slower than the 50 Hz policy tick
    # produces velocity spikes with zeros between them.
    odom_every = max(1, int(round((1.0 / corrupt.odom_rate) / ctrl_dt))) if corrupt.odom_rate > 0 else 1
    last_odom_xy = data.qpos[robot.root_qpos : robot.root_qpos + 2].copy()
    last_odom_tick = 0

    rows = []
    fall_time = None
    start_xy = data.qpos[robot.root_qpos : robot.root_qpos + 2].copy()
    total_ticks = int((settle_s + walk_s) / ctrl_dt)

    for tick in range(total_ticks):
        t = tick * ctrl_dt
        cmd = np.array([vx, 0.0, 0.0]) if t >= settle_s else np.zeros(3)

        R = robot.base_rot(data)
        gravity = R.T @ np.array([0.0, 0.0, -1.0])
        gyro = robot.gyro(data)
        if corrupt.imu_noise > 0.0:
            gyro = gyro + rng.normal(0.0, corrupt.imu_noise, 3)
            gravity = gravity + rng.normal(0.0, corrupt.imu_noise, 3)
        q = robot.q(data)
        dq = robot.dq(data)

        q_obs = q - cfg["default_pose"]
        dq_obs = dq.copy()
        q_obs[list(HEAD_SLOTS)] = 0.0
        dq_obs[list(HEAD_SLOTS)] = 0.0

        speed = float(np.linalg.norm(cmd))
        ph = phase if speed >= cfg["stand_threshold"] else np.array([math.pi, math.pi])
        phase_obs = np.array([math.cos(ph[0]), math.cos(ph[1]), math.sin(ph[0]), math.sin(ph[1])])

        parts = []
        if obs_dim == 82:
            # The 82-obs contract's base linear velocity, as K1WalkPolicy built it: world
            # odometry differentiated, rotated into the body yaw frame, EMA-filtered.
            #   true    -- odometry exact and arriving every tick (what NUSim gives for free)
            #   aliased -- odometry only arrives at odom_rate, so the difference quotient is
            #              a spike train: at 10 Hz, 5x spikes with four zeros between them
            #   zero    -- the channel is dead in CUSTOM, which is the case the hardware
            #              evidence could not rule out
            if corrupt.linvel_mode == "zero":
                linvel_body = np.zeros(2)
            else:
                now_xy = data.qpos[robot.root_qpos : robot.root_qpos + 2].copy()
                if corrupt.linvel_mode == "true" or (tick - last_odom_tick) >= odom_every:
                    dt_odom = ctrl_dt * max(1, tick - last_odom_tick) if corrupt.linvel_mode != "true" else ctrl_dt
                    v_world = (now_xy - last_odom_xy) / dt_odom
                    last_odom_xy, last_odom_tick = now_xy, tick
                else:
                    # No new sample: K1WalkPolicy still differentiates, and gets zero.
                    v_world = np.zeros(2)
                yaw = math.atan2(R[1, 0], R[0, 0])
                c, s = math.cos(-yaw), math.sin(-yaw)
                v_body = np.array([c * v_world[0] - s * v_world[1], s * v_world[0] + c * v_world[1]])
                linvel_body = cfg["linvel_alpha"] * v_body + (1.0 - cfg["linvel_alpha"]) * linvel_body
            parts.append(np.array([linvel_body[0], linvel_body[1], 0.0]))
        parts += [gyro, gravity, cmd, q_obs, dq_obs, last_action.astype(float), phase_obs]
        fresh = np.concatenate(parts)
        if fresh.size != obs_dim:
            raise SystemExit(f"assembled {fresh.size} observations, model wants {obs_dim}")

        # Transport delay, as a linear interpolation over the observation history.
        if obs_hist is None:
            obs_hist = np.tile(fresh, (obs_taps, 1))
        else:
            obs_hist = np.roll(obs_hist, 1, axis=0)
            obs_hist[0] = fresh
        obs = (obs_w @ obs_hist).astype(np.float32)[None, :]

        action = policy.run(None, {input_name: obs})[0][0]
        last_action = action.astype(np.float32)

        phase = np.mod(phase + 2.0 * math.pi * ctrl_dt * cfg["gait_frequency"] + math.pi, 2.0 * math.pi) - math.pi

        q_ref = cfg["default_pose"] + cfg["action_scale"] * last_action
        if cfg["joint_lower"] is not None:
            q_ref = np.clip(q_ref, cfg["joint_lower"], cfg["joint_upper"])
        q_ref[list(HEAD_SLOTS)] = 0.0  # the policy does not own the head
        # Command-side transport delay: a fractional-step blend with the previous target.
        applied_ref = (1.0 - corrupt.action_delay) * q_ref + corrupt.action_delay * prev_q_ref
        prev_q_ref = q_ref
        q_ref = applied_ref

        for _ in range(substeps):
            robot.apply_pd(data, q_ref, kp, kd)
            mujoco.mj_step(model, data)

        (l_fz, l_cop, l_pitch), (r_fz, r_cop, r_pitch) = robot.foot_contacts(data)
        base_z = float(data.qpos[robot.root_qpos + 2])
        up_z = float(R[2, 2])
        rows.append(
            dict(
                t=t,
                cmd_vx=cmd[0],
                base_z=base_z,
                up_z=up_z,
                l_fz=l_fz,
                l_cop_x=l_cop,
                l_pitch=l_pitch,
                r_fz=r_fz,
                r_cop_x=r_cop,
                r_pitch=r_pitch,
                sat=float(np.mean(np.abs(action) > 0.99)),
                knee=float(0.5 * (q[LEFT_KNEE] + q[RIGHT_KNEE])),
            )
        )

        if fall_time is None and (base_z < 0.30 or up_z < 0.5):
            fall_time = t
            break

    if trace_path is not None:
        with trace_path.open("w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    walking = [r for r in rows if r["t"] >= settle_s]
    stance = [(r["l_cop_x"], r["l_pitch"]) for r in walking if r["l_fz"] > 20.0] + [
        (r["r_cop_x"], r["r_pitch"]) for r in walking if r["r_fz"] > 20.0
    ]
    cops = np.array([c for c, _ in stance]) if stance else np.array([np.nan])
    pitches = np.array([p for _, p in stance]) if stance else np.array([np.nan])
    end_xy = data.qpos[robot.root_qpos : robot.root_qpos + 2]

    return dict(
        friction=friction,
        timeconst=timeconst,
        fell=fall_time is not None,
        fall_time=fall_time if fall_time is not None else float("nan"),
        walked_s=max(0.0, (rows[-1]["t"] if rows else 0.0) - settle_s),
        distance=float(np.linalg.norm(end_xy - start_xy)),
        cop_x_mean=float(np.nanmean(cops)),
        cop_x_p95=float(np.nanpercentile(cops, 95)),
        sole_pitch_mean=float(np.nanmean(pitches)),
        sat=float(np.mean([r["sat"] for r in walking])) if walking else float("nan"),
        knee_mean=float(np.mean([r["knee"] for r in walking])) if walking else float("nan"),
    )


def main() -> int:
    here = pathlib.Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policy", required=True, type=pathlib.Path, help="walk policy .onnx")
    parser.add_argument("--config", required=True, type=pathlib.Path, help="K1WalkPolicy.yaml")
    parser.add_argument("--scene", type=pathlib.Path, default=here / "models/k1/k1_scene_flat.xml")
    parser.add_argument("--keyframe", default="ready")
    parser.add_argument("--vx", type=float, default=0.2, help="commanded forward velocity, m/s")
    parser.add_argument("--friction", type=float, nargs="+", default=[1.0, 0.6, 0.4, 0.3, 0.25])
    parser.add_argument("--solref-timeconst", type=float, default=0.02)
    parser.add_argument("--solref-dampratio", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=2.0, help="seconds standing before the command")
    parser.add_argument("--walk", type=float, default=20.0, help="seconds of commanded walking")
    parser.add_argument("--trace-dir", type=pathlib.Path, default=None, help="write a per-tick CSV per run")
    parser.add_argument(
        "--clamp-joints",
        action="store_true",
        help="clamp commanded positions to joint_lower/joint_upper from the config. Off by "
        "default so a run reproduces what the deployed build actually streamed.",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--gait-frequency",
        type=float,
        default=None,
        help="override the config's gait_frequency. Training randomises U(1.25, 1.75), so "
        "the whole range is in distribution and this changes nothing about correctness -- "
        "it tests whether the free-running gait clock is what decouples from the feet.",
    )
    corruption = parser.add_argument_group(
        "deployment corruption",
        "NUSim publishes exact, instantaneous, never-stale state. These make it lie in the "
        "specific ways the real path plausibly does.",
    )
    corruption.add_argument(
        "--linvel-mode",
        choices=["true", "aliased", "zero"],
        default="true",
        help="82-obs contract only: how obs[0:3] is produced (default: true)",
    )
    corruption.add_argument(
        "--odom-rate", type=float, default=50.0, help="odometry publish rate for --linvel-mode aliased"
    )
    corruption.add_argument("--obs-delay", type=float, default=0.0, help="observation transport delay, in 20 ms ticks")
    corruption.add_argument("--action-delay", type=float, default=0.0, help="command transport delay, in 20 ms ticks")
    corruption.add_argument("--imu-noise", type=float, default=0.0, help="gaussian sigma on gyro and projected gravity")
    args = parser.parse_args()

    session = ort.InferenceSession(str(args.policy), providers=["CPUExecutionProvider"])
    obs_dim = session.get_inputs()[0].shape[1]
    if obs_dim not in (79, 82):
        raise SystemExit(f"unsupported observation width {obs_dim}; expected 79 or 82")
    cfg = load_policy_config(args.config)
    if args.gait_frequency is not None:
        cfg["gait_frequency"] = args.gait_frequency
    if not args.clamp_joints:
        cfg["joint_lower"] = None
        cfg["joint_upper"] = None
    if args.trace_dir is not None:
        args.trace_dir.mkdir(parents=True, exist_ok=True)

    corrupt = Corruption(args)
    print(f"policy {args.policy.name}  obs={obs_dim}  scene={args.scene.name}  vx={args.vx} m/s")
    print(f"corruption: {corrupt.describe()}  solref=[{args.solref_timeconst}, {args.solref_dampratio}]")
    print(
        f"{'mu':>6} {'fell':>5} {'fall_t':>7} {'dist':>6} {'cop_x':>7} {'cop_p95':>8} {'sole_p':>7} {'sat':>6} {'knee':>6}"
    )
    results = []
    for mu in args.friction:
        trace = args.trace_dir / f"walk_mu{mu:.2f}.csv" if args.trace_dir else None
        r = run_one(
            args.scene,
            session,
            obs_dim,
            cfg,
            corrupt,
            mu,
            args.solref_timeconst,
            args.solref_dampratio,
            args.vx,
            args.settle,
            args.walk,
            args.keyframe,
            args.seed,
            trace,
        )
        results.append(r)
        print(
            f"{r['friction']:6.2f} {str(r['fell']):>5} {r['fall_time']:7.2f} {r['distance']:6.2f} "
            f"{r['cop_x_mean']:7.4f} {r['cop_x_p95']:8.4f} {r['sole_pitch_mean']:7.4f} "
            f"{r['sat']:6.3f} {r['knee_mean']:6.3f}"
        )

    print("\ncop_x is the centre of pressure in the foot frame; the sole box spans")
    print("[-0.064, +0.116] and its centre is 0.026, so cop_x near 0.10 is tip-toe.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
