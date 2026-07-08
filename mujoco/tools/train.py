"""./b train <policy.py> [args] — train a locomotion policy with the MJX framework.

`./b train walk.py` runs mujoco/training/train.py against training/policies/walk.py
(the Policy subclass whose reward/commands define that policy). Extra args pass
through (e.g. --iterations, --num-envs, --render). Training runs on the host in a
Python venv (JAX/MJX GPU) — see mujoco/training/README.md for setup; it is NOT the
docker sim build.
"""
import os
import subprocess
import sys

import b


def register(command):
    command.description = "Train a locomotion policy (MJX/JAX)"
    command.add_argument("policy", help="policy script under training/policies/, e.g. walk.py")
    command.add_argument("args", nargs="*", help="arguments forwarded to training/train.py")


def run(policy, args, **kwargs):
    train_py = os.path.join(b.mujoco_dir, "training", "train.py")
    if not os.path.isfile(train_py):
        sys.exit("training/train.py not found — the MJX training framework is not set up yet")
    policy_path = os.path.join("policies", policy)
    python = os.environ.get("K1SIM_TRAIN_PYTHON", sys.executable)
    rc = subprocess.call(
        [python, train_py, "--policy", policy_path, *args],
        cwd=os.path.join(b.mujoco_dir, "training"),
    )
    sys.exit(rc)
