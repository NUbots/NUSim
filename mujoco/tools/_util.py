"""Shared helpers for ./b tool modules."""

import os
import platform
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import b


def backend():
    """Use native builds on macOS, preserving Docker as the Linux default."""
    value = os.environ.get("K1SIM_BACKEND", "native" if platform.system() == "Darwin" else "docker")
    if value not in ("native", "docker"):
        sys.exit("error: K1SIM_BACKEND must be native or docker")
    return value


def build_dir():
    return os.environ.get("K1SIM_BUILD_DIR", "build-native" if backend() == "native" else "build-docker")


def _native(args, env):
    """Run the same configure/build/run workflow without a container."""
    source = Path(b.mujoco_dir)
    build = source / build_dir()
    command, *rest = args

    def call(argv):
        subprocess.run(argv, cwd=source, env=env, check=True)

    def configure():
        call(["cmake", "-S", str(source), "-B", str(build), "-GNinja", *shlex.split(env.get("K1SIM_CMAKE_ARGS", ""))])

    def compile_targets(targets):
        if not (build / "build.ninja").is_file():
            configure()
        argv = ["cmake", "--build", str(build), "--parallel", env.get("JOBS", str(os.cpu_count() or 1))]
        if targets:
            argv += ["--target", *targets]
        call(argv)

    if command == "configure":
        configure()
    elif command == "ccmake":
        if not (build / "CMakeCache.txt").is_file():
            configure()
        call(["ccmake", "-S", str(source), "-B", str(build)])
    elif command == "build":
        compile_targets(rest)
    elif command == "clean":
        if build.resolve() == source or build.resolve() in source.parents:
            sys.exit("error: the build directory must not be the source directory or its parent")
        shutil.rmtree(build, ignore_errors=True)
        print(f"removed {build}")
    elif command == "test":
        call(["ctest", "--test-dir", str(build), "--output-on-failure", *rest])
    elif command == "run":
        binary, *binary_args = rest
        executable = build / binary
        if not executable.is_file():
            compile_targets([])
        # Replace the dispatcher so signals reach the simulator directly.
        os.chdir(source)
        os.execvpe(str(executable), [str(executable), *binary_args], env)
    else:
        sys.exit(f"error: {command} requires K1SIM_BACKEND=docker")


def k1sim(*args, env=None):
    """Dispatch to the selected backend; exit with its return code on failure."""
    full_env = dict(os.environ)
    if env:
        full_env.update(env)
    if backend() == "native" and args[0] != "image":
        try:
            _native(args, full_env)
        except subprocess.CalledProcessError as e:
            sys.exit(e.returncode)
        except FileNotFoundError as e:
            sys.exit(f"error: {e.filename} not found; check the native prerequisites in docs/K1_MUJOCO_SETUP.md")
        return
    cmd = [b.k1sim_sh, *args]
    rc = subprocess.call(cmd, env=full_env)
    if rc != 0:
        sys.exit(rc)


def _scalar(text):
    """A YAML scalar or flow list: int, float, true/false, [a, b], or a string."""
    text = text.split(" #", 1)[0].strip()
    if text.startswith("[") and text.endswith("]"):
        return [_scalar(item) for item in text[1:-1].split(",") if item.strip()]
    if text in ("true", "false"):
        return text == "true"
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return text.strip("\"'")


def config_section(file, key):
    """A top-level mapping from a config/ file, nested by indentation: mappings, scalars and flow lists only, which
    is all ./b reads (the host's Python has no YAML parser)."""
    lines = (Path(b.mujoco_dir) / "config" / file).read_text().splitlines()
    start = next(i for i, line in enumerate(lines) if line.split("#", 1)[0].rstrip() == f"{key}:")
    root = {}
    stack = [[None, root]]  # [indent of the mapping's keys (set by its first), mapping]
    for line in lines[start + 1 :]:
        content = "" if line.lstrip().startswith("#") else line.split(" #", 1)[0].rstrip()
        if not content.strip():
            continue
        indent = len(content) - len(content.lstrip())
        if indent == 0:
            break
        while stack[-1][0] is not None and indent < stack[-1][0]:
            stack.pop()
        if stack[-1][0] is None:
            stack[-1][0] = indent
        name, _, value = content.strip().partition(":")
        mapping = stack[-1][1]
        if value.strip():
            mapping[name] = _scalar(value)
        else:
            mapping[name] = {}
            stack.append([None, mapping[name]])
    return root
