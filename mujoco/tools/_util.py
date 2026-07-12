"""Shared helpers for ./b tool modules."""
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
