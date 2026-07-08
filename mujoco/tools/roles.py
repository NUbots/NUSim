"""./b roles — list the sim roles and whether each is enabled.

Reads mujoco/roles/**/*.role and, if a build exists, the ROLE_* state from its
CMakeCache. Toggle with `./b configure --set-role/--unset-role`, or `-i` for ccmake.
"""
import glob
import os

import b


def register(command):
    command.description = "List sim roles and their enabled/disabled state"


def run(**kwargs):
    roles_dir = os.path.join(b.mujoco_dir, "roles")
    cache = os.path.join(b.mujoco_dir, os.environ.get("K1SIM_BUILD_DIR", "build-docker"), "CMakeCache.txt")

    state = {}
    if os.path.isfile(cache):
        for line in open(cache):
            if line.startswith("ROLE_") and ":BOOL=" in line:
                var, val = line.strip().split(":BOOL=")
                state[var] = val

    print("roles (./b run <role>):")
    for f in sorted(glob.glob(os.path.join(roles_dir, "**", "*.role"), recursive=True)):
        rel = os.path.relpath(f, roles_dir)[:-5]            # sim/soccer
        var = "ROLE_" + rel.replace("/", "-")
        print(f"  {rel:20} {state.get(var, 'ON (default)')}")
