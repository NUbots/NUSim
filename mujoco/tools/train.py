"""./b train <policy.py> [args] — train a locomotion policy with the MJX framework.

`./b train walk.py` runs mujoco/training/train.py against training/policies/walk.py
(the Policy subclass whose reward/commands define that policy). Extra args pass
through (e.g. --iterations, --num-envs, --render). Training runs on the host in a
Python venv (JAX/MJX GPU) — see mujoco/training/README.md for setup; it is NOT the
docker sim build.
"""
import argparse
import glob
import os
import subprocess
import sys

import b


def _cuda_env(python):
    """LD_LIBRARY_PATH with the venv's bundled nvidia-*-cu12 libs prepended.

    jax[cuda12]'s pip wheels ship their own CUDA libs, but a system CUDA on
    LD_LIBRARY_PATH (e.g. /usr/local/cuda) shadows them with an older, mismatched
    libcusparse → 'cuSPARSE not found' and a silent CPU fallback. Prepending the
    wheel dirs makes the loader pick the versions jax was built against.
    """
    env = dict(os.environ)
    venv = os.path.dirname(os.path.dirname(python))  # .../.venv/bin/python -> .../.venv
    nvidia_libs = sorted(glob.glob(os.path.join(venv, "lib", "python*", "site-packages", "nvidia", "*", "lib")))
    if nvidia_libs:
        env["LD_LIBRARY_PATH"] = os.pathsep.join(nvidia_libs + [env.get("LD_LIBRARY_PATH", "")]).rstrip(os.pathsep)
    return env


def register(command):
    command.description = "Train a locomotion policy (MJX/JAX)"
    command.add_argument("policy", help="policy script under training/policies/, e.g. walk.py")
    # REMAINDER (not "*"): pass flags like --render / --iterations straight through
    # to training/train.py instead of argparse trying to parse them.
    command.add_argument("args", nargs=argparse.REMAINDER, help="arguments forwarded to training/train.py")


def run(policy, args, **kwargs):
    train_py = os.path.join(b.mujoco_dir, "training", "train.py")
    if not os.path.isfile(train_py):
        sys.exit("training/train.py not found — the MJX training framework is not set up yet")
    policy_path = os.path.join("policies", policy)
    # Prefer the uv-managed venv (populated by `uv sync --extra train`, which holds
    # jax/mjx/brax); fall back to K1SIM_TRAIN_PYTHON, then whatever ran ./b.
    venv_python = os.path.join(b.repo_dir, ".venv", "bin", "python")
    default_python = venv_python if os.path.isfile(venv_python) else sys.executable
    python = os.environ.get("K1SIM_TRAIN_PYTHON", default_python)
    rc = subprocess.call(
        [python, train_py, "--policy", policy_path, *args],
        cwd=os.path.join(b.mujoco_dir, "training"),
        env=_cuda_env(python),
    )
    sys.exit(rc)
