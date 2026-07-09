"""Shared helpers for ./b tool modules."""
import glob
import os
import subprocess
import sys

import b


def k1sim(*args, env=None):
    """Invoke docker/k1sim.sh with args; exit with its return code on failure."""
    cmd = [b.k1sim_sh, *args]
    full_env = dict(os.environ)
    if env:
        full_env.update(env)
    rc = subprocess.call(cmd, env=full_env)
    if rc != 0:
        sys.exit(rc)


def _venv_python():
    return os.path.join(b.repo_dir, ".venv", "bin", "python")


def _cuda_ld_path(env):
    """Prepend the venv's bundled nvidia-*-cu12 libs so torch/mjlab pick the versions
    they were built against instead of a shadowing system CUDA on LD_LIBRARY_PATH."""
    nvidia = sorted(glob.glob(os.path.join(b.repo_dir, ".venv", "lib", "python*",
                                           "site-packages", "nvidia", "*", "lib")))
    if nvidia:
        env["LD_LIBRARY_PATH"] = os.pathsep.join(nvidia + [env.get("LD_LIBRARY_PATH", "")]).rstrip(os.pathsep)
    return env


def mjlab_run(script, task, extra_args):
    """Run mjlab's train/play CLI on a K1 task via the uv venv.

    Imports training_mjlab.tasks first (registers all K1 tasks in
    mjlab's registry), then delegates to mjlab.scripts.<script>.main(). Needs
    NUSim/mujoco on PYTHONPATH (training_mjlab lives there; the dir can't be a
    package or it would shadow the installed `mujoco` bindings).
    """
    env = dict(os.environ)
    env["PYTHONPATH"] = b.mujoco_dir + os.pathsep + env.get("PYTHONPATH", "")
    # mjlab's train reads CUDA_VISIBLE_DEVICES ("" => CPU); default to GPU 0.
    env.setdefault("CUDA_VISIBLE_DEVICES", "0")
    _cuda_ld_path(env)
    # Import the tasks package (registers every K1 task with mjlab's registry) then
    # hand off to mjlab's tyro CLI, which picks the task by id from that registry.
    code = ("import training_mjlab.tasks; "
            f"from mjlab.scripts.{script} import main; main()")
    rc = subprocess.call([_venv_python(), "-c", code, task, *extra_args], env=env)
    sys.exit(rc)
