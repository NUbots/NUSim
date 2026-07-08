#!/usr/bin/env python3
"""Contract checks for the vendored K1 model + RoboCup scene (workstream A / M1).

Loads mujoco/models/k1/k1_scene_robocup.xml and asserts:
  1. the model compiles;
  2. there are exactly 22 actuators, named (in order) exactly as the frozen
     JointIndexK1 contract in mujoco/shared/k1/JointIndex.hpp;
  3. sensors "orientation", "angular-velocity", "acceleration",
     "linear-velocity" and site "imu" exist;
  4. keyframes "ready", "lying_front", "lying_back" exist;
  5. a 1 m ball-drop test's first-bounce apex height is within 15% of
     0.76^2 * drop_height (Webots' RoboCup ball/grass bounce coefficient);
  6. (informational) a plain per-joint PD loop, seeded from
     mujoco/config/gains.yaml's kp/kd/ready_pose, holds the "ready" stance
     for 10 s. This is not expected to succeed with joint-space PD alone
     (no whole-body balance strategy) so a failure here prints WARN and does
     NOT affect the script's exit code -- gain/controller tuning is
     workstream B's milestone, not this model-and-scene contract.

Exit code: 0 if checks 1-5 all pass, 1 otherwise. Prints a PASS/FAIL/WARN
summary line per check.
"""
import math
import os
import re
import sys

import numpy as np

try:
    import yaml
except ImportError:  # pragma: no cover - pyyaml is expected to be present
    yaml = None

import mujoco

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MUJOCO_DIR = os.path.abspath(os.path.join(THIS_DIR, "..", ".."))
SCENE_XML = os.path.join(MUJOCO_DIR, "models", "k1", "k1_scene_robocup.xml")
JOINT_INDEX_HPP = os.path.join(MUJOCO_DIR, "shared", "k1", "JointIndex.hpp")
GAINS_YAML = os.path.join(MUJOCO_DIR, "config", "gains.yaml")

BALL_RADIUS = 0.0785
DROP_BASE_Z = 1.0785  # ball center height for the 1 m drop test (1.0 m above floor)
BOUNCE_COEFF = 0.76
TARGET_RATIO = BOUNCE_COEFF ** 2  # ~0.5776
RATIO_TOLERANCE = 0.15

results = []  # list of (status, message) where status in {"PASS", "FAIL", "WARN"}


def record(status, message):
    results.append((status, message))
    print(f"[{status}] {message}")


def parse_joint_names(header_path):
    """Extract the JOINT_NAMES string array from JointIndex.hpp (the frozen
    JointIndexK1 contract), without needing to compile any C++."""
    text = open(header_path, "r").read()
    m = re.search(r"JOINT_NAMES\s*=\s*\{(.*?)\};", text, re.DOTALL)
    if not m:
        raise RuntimeError(f"could not find JOINT_NAMES array in {header_path}")
    body = m.group(1)
    names = re.findall(r'"([^"]+)"', body)
    if not names:
        raise RuntimeError(f"JOINT_NAMES array in {header_path} contained no strings")
    return names


def quat_tilt_deg(quat_wxyz):
    """Angle (deg) between the body's local +z axis and world +z."""
    xmat = np.zeros(9)
    mujoco.mju_quat2Mat(xmat, quat_wxyz)
    xmat = xmat.reshape(3, 3)
    return math.degrees(math.acos(np.clip(xmat[2, 2], -1.0, 1.0)))


def main():
    overall_ok = True

    # -- 1. compile -----------------------------------------------------
    try:
        model = mujoco.MjModel.from_xml_path(SCENE_XML)
        data = mujoco.MjData(model)
        record("PASS", f"model compiles ({SCENE_XML})")
    except Exception as exc:  # noqa: BLE001 - want to report any compile error
        record("FAIL", f"model failed to compile: {exc}")
        print_summary(False)
        sys.exit(1)

    # -- 2. actuators -----------------------------------------------------
    try:
        expected_names = parse_joint_names(JOINT_INDEX_HPP)
    except Exception as exc:  # noqa: BLE001
        record("FAIL", f"could not parse JointIndex.hpp: {exc}")
        expected_names = []
        overall_ok = False

    actual_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
    ]
    if model.nu != 22:
        record("FAIL", f"expected 22 actuators, got {model.nu}")
        overall_ok = False
    elif actual_names == expected_names:
        record("PASS", f"22 actuators present, names match JointIndexK1 order exactly")
    else:
        record(
            "FAIL",
            f"actuator names/order mismatch vs JointIndex.hpp:\n"
            f"  expected: {expected_names}\n"
            f"  actual:   {actual_names}",
        )
        overall_ok = False

    # -- 3. sensors + site -----------------------------------------------
    for sensor_name in ("orientation", "angular-velocity", "acceleration", "linear-velocity"):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sid == -1:
            record("FAIL", f"sensor '{sensor_name}' not found")
            overall_ok = False
        else:
            record("PASS", f"sensor '{sensor_name}' present (id={sid})")

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")
    if site_id == -1:
        record("FAIL", "site 'imu' not found")
        overall_ok = False
    else:
        record("PASS", f"site 'imu' present (id={site_id})")

    # -- 4. keyframes -----------------------------------------------------
    for key_name in ("ready", "lying_front", "lying_back"):
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key_name)
        if kid == -1:
            record("FAIL", f"keyframe '{key_name}' not found")
            overall_ok = False
        else:
            record("PASS", f"keyframe '{key_name}' present (id={kid})")

    # -- 5. ball drop / bounce test ---------------------------------------
    try:
        ratio = run_ball_drop_test(model, data)
        lower = TARGET_RATIO * (1 - RATIO_TOLERANCE)
        upper = TARGET_RATIO * (1 + RATIO_TOLERANCE)
        if lower <= ratio <= upper:
            record(
                "PASS",
                f"ball drop: first-bounce apex ratio {ratio:.4f} within 15% of "
                f"target {TARGET_RATIO:.4f} (0.76^2) [{lower:.4f}, {upper:.4f}]",
            )
        else:
            record(
                "FAIL",
                f"ball drop: first-bounce apex ratio {ratio:.4f} outside 15% band "
                f"[{lower:.4f}, {upper:.4f}] around target {TARGET_RATIO:.4f}",
            )
            overall_ok = False
    except Exception as exc:  # noqa: BLE001
        record("FAIL", f"ball drop test raised an exception: {exc}")
        overall_ok = False

    # -- 6. PD stability sanity (informational; never fails the script) ---
    try:
        pd_ok, pd_msg = run_pd_stability_check(model, data)
        record("PASS" if pd_ok else "WARN", pd_msg)
    except Exception as exc:  # noqa: BLE001
        record("WARN", f"PD stability check raised an exception (not fatal): {exc}")

    print_summary(overall_ok)
    sys.exit(0 if overall_ok else 1)


def run_ball_drop_test(model, data):
    """Reset, drop the ball from 1 m above the floor, return the first-bounce
    apex height ratio (apex_height / drop_height)."""
    mujoco.mj_resetData(model, data)

    ball_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ball")
    ball_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ball")
    if ball_body_id == -1 or ball_geom_id == -1:
        raise RuntimeError("scene has no 'ball' body/geom")

    joint_id = model.body_jntadr[ball_body_id]
    if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError("'ball' body does not have a free joint")
    qadr = model.jnt_qposadr[joint_id]

    # The ball body frame is at the origin with the sphere geom offset
    # locally to (1.38, 0, 0.0785) -- see k1_scene_robocup.xml's header
    # comment. Setting the body-frame z so the *geom* sits at DROP_BASE_Z:
    geom_local_z = model.geom_pos[ball_geom_id][2]
    data.qpos[qadr : qadr + 3] = [0.0, 0.0, DROP_BASE_Z - geom_local_z]
    data.qpos[qadr + 3 : qadr + 7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    max_time = 3.0
    nsteps = int(max_time / dt)
    z = np.empty(nsteps)
    for i in range(nsteps):
        mujoco.mj_step(model, data)
        z[i] = data.geom_xpos[ball_geom_id][2]

    vz = np.diff(z)
    bounce_idx = next((i for i in range(1, len(vz)) if vz[i - 1] < 0 <= vz[i]), None)
    if bounce_idx is None:
        raise RuntimeError("ball never bounced off the floor within the simulated window")
    apex_idx = next(
        (i for i in range(bounce_idx + 1, len(vz)) if vz[i - 1] > 0 >= vz[i]), None
    )
    if apex_idx is None:
        raise RuntimeError("ball bounced but no subsequent apex was found")

    drop_height = DROP_BASE_Z - BALL_RADIUS
    apex_height = z[apex_idx] - BALL_RADIUS
    return apex_height / drop_height


def run_pd_stability_check(model, data):
    """Set the 'ready' keyframe, drive a plain per-joint PD loop using
    gains.yaml's kp/kd/ready_pose (JointIndexK1 order) for 10 s, and check
    that the base height and tilt stay within bounds. Returns (ok, message);
    `ok=False` is only ever a WARN, never fails the overall script."""
    if yaml is None:
        return False, "PyYAML not available; skipped PD stability check"

    with open(GAINS_YAML, "r") as f:
        gains = yaml.safe_load(f)
    kp = np.array(gains["kp"], dtype=float)
    kd = np.array(gains["kd"], dtype=float)
    ready_pose = np.array(gains["ready_pose"], dtype=float)
    if not (len(kp) == len(kd) == len(ready_pose) == model.nu):
        return False, (
            f"gains.yaml array lengths ({len(kp)}/{len(kd)}/{len(ready_pose)}) "
            f"don't match model.nu ({model.nu}); skipped PD stability check"
        )

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "ready")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    qpos_adr = np.array([model.jnt_qposadr[model.actuator_trnid[i, 0]] for i in range(model.nu)])
    dof_adr = np.array([model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(model.nu)])

    dt = model.opt.timestep
    sim_time = 10.0
    nsteps = int(sim_time / dt)
    min_z, max_z = math.inf, -math.inf
    max_tilt = 0.0
    for _ in range(nsteps):
        q = data.qpos[qpos_adr]
        qd = data.qvel[dof_adr]
        data.ctrl[:] = kp * (ready_pose - q) - kd * qd
        mujoco.mj_step(model, data)
        z = data.qpos[2]
        min_z, max_z = min(min_z, z), max(max_z, z)
        max_tilt = max(max_tilt, quat_tilt_deg(data.qpos[3:7]))

    height_ok = 0.45 <= min_z and max_z <= 0.60
    tilt_ok = max_tilt < 15.0
    ok = height_ok and tilt_ok
    msg = (
        f"PD stability ({sim_time:.0f}s, official gains.yaml kp/kd/ready_pose): "
        f"height range [{min_z:.3f}, {max_z:.3f}] m (want [0.45, 0.60]), "
        f"max tilt {max_tilt:.1f} deg (want < 15) -> "
        + ("held" if ok else "did NOT hold (expected: joint-space PD has no balance "
           "strategy; full stabilization is workstream B's PD/controller milestone)")
    )
    return ok, msg


def print_summary(overall_ok):
    n_pass = sum(1 for s, _ in results if s == "PASS")
    n_warn = sum(1 for s, _ in results if s == "WARN")
    n_fail = sum(1 for s, _ in results if s == "FAIL")
    print("-" * 72)
    print(f"check_model.py summary: {n_pass} PASS, {n_warn} WARN, {n_fail} FAIL")
    print("OVERALL:", "PASS" if overall_ok else "FAIL")


if __name__ == "__main__":
    main()
