#!/usr/bin/env python3
"""`./b` command dispatcher (a light port of NUbots' nuclear/b.py).

Subcommands are discovered dynamically: `./b <a> <b> ...` resolves to the module
mujoco/tools/<a>/<b>.py (longest path match wins). Each tool module must define
`register(command)` (add argparse args) and `run(**kwargs)`. Directories with an
__init__.py are also candidates. Tools import `b` for shared paths (b.repo_dir,
b.mujoco_dir, b.k1sim_sh).
"""
import argparse
import importlib.util
import os
import sys

mujoco_dir = os.path.dirname(os.path.abspath(__file__))
repo_dir = os.path.dirname(mujoco_dir)
tools_dir = os.path.join(mujoco_dir, "tools")
k1sim_sh = os.path.join(mujoco_dir, "docker", "k1sim.sh")

# Expose shared paths to tool modules via `import b`.
sys.modules["b"] = sys.modules[__name__]
sys.path.insert(0, tools_dir)


def _candidates():
    """Map dotted command paths to their tool file, e.g. ('run',) -> tools/run.py."""
    found = {}
    for dirpath, dirnames, filenames in os.walk(tools_dir):
        rel = os.path.relpath(dirpath, tools_dir)
        parts = [] if rel == "." else rel.split(os.sep)
        for fn in filenames:
            if fn.endswith(".py") and fn != "__init__.py" and not fn.startswith("_"):
                found[tuple(parts + [fn[:-3]])] = os.path.join(dirpath, fn)
    return found


def _load(path):
    spec = importlib.util.spec_from_file_location("_k1sim_tool", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    candidates = _candidates()
    argv = sys.argv[1:]

    # Longest command-path match first.
    match = None
    for key in sorted(candidates, key=len, reverse=True):
        if list(key) == argv[: len(key)]:
            match = key
            break

    if match is None:
        names = sorted({k[0] for k in candidates})
        print("usage: ./b <command> [args]\ncommands: " + ", ".join(names))
        sys.exit(0 if not argv else 1)

    module = _load(candidates[match])
    rest = argv[len(match):]
    parser = argparse.ArgumentParser(prog="./b " + " ".join(match))
    if hasattr(module, "register"):
        module.register(parser)
    args = parser.parse_args(rest)
    module.run(**vars(args))


if __name__ == "__main__":
    main()
