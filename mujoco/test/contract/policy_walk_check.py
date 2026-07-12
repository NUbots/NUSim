"""Sim2sim spot-check: roll a trained playground ONNX policy on NUSim's vendored
K1 model with a pure-python replica of PolicyBackend (same obs/action/PD math,
see docs/OBS_ACTION_CONTRACT.md) and report walking metrics.

This catches training/deployment parity bugs (gains, default pose, forcerange,
obs order) without the full NUClear + DDS stack.

Usage:
    python policy_walk_check.py --onnx /path/to/k1_walk_policy.onnx \
        [--vx 0.3] [--seconds 8]

Exit 0 iff the robot stays upright and makes forward progress.
"""
import argparse
import pathlib

import mujoco
import numpy as np
import onnxruntime as rt

HERE = pathlib.Path(__file__).resolve().parent  # mujoco/test/contract
MUJOCO_DIR = HERE.parent.parent

# JointIndexK1 order == model joint order. Values mirror config/locomotion.yaml policy block.
KP = np.array([3.948] * 10 + [30.201, 21.448, 17.846, 60.402, 35.692, 35.692] * 2)
KD = np.array([0.251] * 10 + [3.605, 2.56, 2.13, 4.807, 4.26, 4.26] * 2)
ACTION_SCALE = np.array([0.3799] * 2 + [0.8865] * 8
                        + [0.5629, 0.8859, 0.5365, 0.4636, 0.2683, 0.2683] * 2)
DEFAULT_POSE = np.array([0, 0, 0, -1.3, 0, 0, 0, 1.3, 0, 0]
                        + [-0.2, 0, 0, 0.4, -0.2, 0] * 2, dtype=np.float64)
GAIT_FREQ = 1.5
INFERENCE_DIVISOR = 20
STAND_THRESHOLD = 0.01


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx", required=True)
    parser.add_argument("--vx", type=float, default=0.3)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--vyaw", type=float, default=0.0)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path((MUJOCO_DIR / "models/k1/k1_scene_flat.xml").as_posix())
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready"))
    # Start from the training crouch, not the straight-leg ready pose.
    data.qpos[7:] = DEFAULT_POSE
    data.qpos[2] = 0.543  # training keyframe base height
    mujoco.mj_forward(model, data)

    sess = rt.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    sens = {name: model.sensor(name).adr[0] for name in ("linear-velocity", "angular-velocity")}
    cmd = np.array([args.vx, args.vy, args.vyaw], dtype=np.float64)
    phase = np.array([0.0, np.pi])
    phase_dt = 2 * np.pi * GAIT_FREQ * (INFERENCE_DIVISOR * model.opt.timestep)
    last_action = np.zeros(22, dtype=np.float64)

    n_steps = int(args.seconds / model.opt.timestep)
    max_tilt = 0.0
    for step in range(n_steps):
        if step % INFERENCE_DIVISOR == 0:
            quat = data.qpos[3:7]
            neg = np.array([quat[0], -quat[1], -quat[2], -quat[3]])
            grav = np.zeros(3)
            mujoco.mju_rotVecQuat(grav, np.array([0.0, 0.0, -1.0]), neg)
            ph = phase if np.linalg.norm(cmd) >= STAND_THRESHOLD else np.ones(2) * np.pi
            obs = np.hstack([
                data.sensordata[sens["linear-velocity"]:sens["linear-velocity"] + 3],
                data.sensordata[sens["angular-velocity"]:sens["angular-velocity"] + 3],
                grav,
                cmd,
                data.qpos[7:29] - DEFAULT_POSE,
                data.qvel[6:28],
                last_action,
                np.cos(ph), np.sin(ph),
            ]).astype(np.float32)
            last_action = sess.run(["continuous_actions"], {"obs": obs.reshape(1, -1)})[0][0].astype(np.float64)
            phase = np.fmod(phase + phase_dt + np.pi, 2 * np.pi) - np.pi

        q_ref = DEFAULT_POSE + ACTION_SCALE * last_action
        tau = KP * (q_ref - data.qpos[7:29]) - KD * data.qvel[6:28]
        lo, hi = model.actuator_forcerange[:, 0], model.actuator_forcerange[:, 1]
        data.ctrl[:22] = np.clip(tau, lo[:22], hi[:22])
        mujoco.mj_step(model, data)

        up = np.zeros(3)
        quat = data.qpos[3:7]
        mujoco.mju_rotVecQuat(up, np.array([0.0, 0.0, 1.0]), quat)
        max_tilt = max(max_tilt, float(np.degrees(np.arccos(np.clip(up[2], -1, 1)))))
        if up[2] < 0.5:  # > 60 deg tilt = fallen
            print(f"FELL at t={step * model.opt.timestep:.2f}s (tilt {max_tilt:.0f} deg), "
                  f"x={data.qpos[0]:.2f}m")
            raise SystemExit(1)

    x, y = data.qpos[0], data.qpos[1]
    expected = args.vx * args.seconds
    print(f"upright for {args.seconds}s, max tilt {max_tilt:.1f} deg")
    print(f"travelled x={x:.2f}m y={y:.2f}m (command vx={args.vx} => expected ~{expected:.2f}m)")
    ok = x > 0.5 * expected if args.vx > 0 else True
    print("PASS" if ok else "FAIL: insufficient forward progress")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
