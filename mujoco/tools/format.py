#!/usr/bin/env python3
#
# MIT License
#
# Copyright (c) 2017 NUbots
#
# This file is part of the NUSim codebase.
# See https://github.com/NUbots/NUSim for further info.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
"""./b format — run the formatters over the codebase (a port of NUbots' tools/format.py).

    ./b format             format files that differ from origin/main
    ./b format --all       format every tracked file
    ./b format --check     print a diff instead of writing, exit 1 if anything differs
    ./b format '*.cpp'     limit to files matching a glob

Differences from the NUbots original, all forced by NUSim's plumbing rather than taste:
  * the formatters run on the host out of the uv project environment, not inside the
    docker image (NUSim's ./b is a host-side stdlib-only dispatcher, see mujoco/b.py),
    so there is no @run_on_docker decorator
  * files are formatted on a thread pool rather than a process pool: b.py loads tool
    modules under a name that is not in sys.modules, so a forked child cannot unpickle
    the work function. The work is all subprocess calls, so threads lose nothing
  * --check diffs with difflib instead of shelling out to colordiff, which is not a
    dependency NUSim otherwise has
  * no licence-header formatter (it is commented out upstream) and therefore no pygit2
  * no eslint/prettier (no JavaScript in this repo)

The formatter versions are pinned in pyproject.toml to the ones NUbots uses, so a given
file formats identically in both repos.
"""
import difflib
import os
import shutil
import sys
import tempfile
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from fnmatch import fnmatch
from subprocess import DEVNULL, PIPE, STDOUT, CalledProcessError
from subprocess import run as sp_run

import b

VENV_BIN = os.path.join(b.repo_dir, ".venv", "bin")


def _sync_env():
    """Create/update the uv project environment holding the pinned formatters.

    --project is required: uv >= 0.12 otherwise walks up from the cwd and can pick a
    different pyproject.toml, landing in an environment where the tools are missing.
    """
    if not os.path.isdir(VENV_BIN):
        print("Setting up the formatter environment (uv sync)...")
    sp_run(["uv", "sync", "--project", b.repo_dir, "--quiet"], check=True, stdout=DEVNULL)


def _tool(name):
    """Absolute path to a pinned formatter, so we don't pay uv's startup cost per file."""
    path = os.path.join(VENV_BIN, name)
    if os.path.isfile(path):
        return path
    found = shutil.which(name)
    if found is None:
        sys.exit(f"error: {name} not found. Run `uv sync` in {b.repo_dir}.")
    return found


# The extensions that are handled by the various formatters
formatters = OrderedDict()
formatters["clang-format"] = {
    "format": [["clang-format", "-i", "-style=file", "{path}"]],
    "include": ["*.h", "*.c", "*.cc", "*.cxx", "*.cpp", "*.hpp", "*.ipp", "*.frag", "*.glsl", "*.vert", "*.proto"],
    # idl_gen/ is fastddsgen output, regenerated on every build (see cmake/IdlGen.cmake);
    # formatting it would be overwritten and would obscure diffs against the generator.
    "exclude": ["mujoco/idl_gen/*", "**/idl_gen/*"],
}
formatters["cmake-format"] = {
    "format": [["cmake-format", "--in-place", "{path}"]],
    "include": ["*.cmake", "*.role", "CMakeLists.txt", "**/CMakeLists.txt"],
    "exclude": [],
}
formatters["isort"] = {
    "format": [["isort", "--quiet", "{path}"]],
    "include": ["*.py"],
    "exclude": [],
}
formatters["black"] = {
    "format": [["black", "--quiet", "{path}"]],
    "include": ["*.py"],
    "exclude": [],
}


def _do_format(path, verbose, check=True):
    """Format one file in a temp copy; either diff it against the original or replace it."""
    text = ""
    success = True
    try:
        # Find the correct formatter and format the file
        formatter = []
        formatter_names = []
        for name, fmt in formatters.items():
            if (any(fnmatch(path, pattern) for pattern in fmt["include"])) and (
                all(not fnmatch(path, pattern) for pattern in fmt["exclude"])
            ):
                formatter_names.append(name)
                formatter.extend(fmt["format"])

        # If we don't have a formatter then skip this file
        if len(formatter) == 0:
            return f"Skipping {path} as it does not match any of the formatters\n" if verbose >= 1 else "", True

        text = f"Formatting {path} with {', '.join(formatter_names)}\n"

        # Format a copy, so a formatter that fails half way can't leave the file mangled.
        with tempfile.TemporaryDirectory(dir=os.path.dirname(path) or ".") as tmp_dir:
            output_path = os.path.join(tmp_dir, os.path.basename(path))
            shutil.copy(path, output_path)

            tool_text = ""
            for c in formatter:
                cmd = [_tool(c[0])] + [arg.format(path=output_path) for arg in c[1:]]
                if verbose >= 2:
                    text += f"\t$ {' '.join(cmd)}\n"
                tool_text += sp_run(cmd, stderr=STDOUT, stdout=PIPE, check=True).stdout.decode("utf-8")

            if verbose >= 1 and tool_text:
                text += tool_text

            with open(path, "r", encoding="utf-8", errors="replace") as f:
                original = f.readlines()
            with open(output_path, "r", encoding="utf-8", errors="replace") as f:
                formatted = f.readlines()

            if original == formatted:
                return ("" if verbose == 0 else text), True

            if check:
                text += "".join(difflib.unified_diff(original, formatted, fromfile=path, tofile=f"{path} (formatted)"))
                success = False
            else:
                shutil.copy(output_path, path)

    except CalledProcessError as e:
        text += e.output.decode("utf-8").strip()
        success = False

    return text, success


def register(command):
    command.description = "Format the code in the codebase (clang-format, cmake-format, isort, black)"

    command.add_argument("-v", "--verbose", action="count", default=0, help="Print the output of the formatters")
    command.add_argument(
        "-a",
        "--all",
        dest="format_all",
        action="store_true",
        default=False,
        help="Include unmodified files, as well as modified files, compared to main.",
    )
    command.add_argument(
        "-c",
        "--check",
        dest="check",
        action="store_true",
        default=False,
        help="Check that files conform to formatting requirements",
    )
    command.add_argument("globs", nargs="*", help="Globs with which to limit the files to format")


def run(verbose, check, format_all, globs, **kwargs):
    os.chdir(b.repo_dir)
    _sync_env()

    # Every tracked file, or just the ones that differ from main.
    if format_all:
        files = sp_run(["git", "ls-files"], stdout=PIPE, check=True).stdout.decode("utf-8").splitlines()
    else:
        base = "origin/main" if _has_ref("origin/main") else "main"
        files = (
            sp_run(["git", "diff", "--name-only", base], stdout=PIPE, check=True).stdout.decode("utf-8").splitlines()
        )

    # git diff can name deleted files
    files = [f for f in files if os.path.isfile(f)]

    if len(globs) != 0:
        files = [f for f in files if any(fnmatch(f, g) for g in globs)]

    if not files:
        print("No files to format")
        return

    success = True
    with ThreadPoolExecutor(max_workers=os.cpu_count()) as pool:
        for r, s in pool.map(lambda f: _do_format(f, verbose=verbose, check=check), files):
            sys.stdout.write(r)
            success = success and s

    sys.exit(0 if success else 1)


def _has_ref(ref):
    return sp_run(["git", "rev-parse", "--verify", "--quiet", ref], stdout=PIPE).returncode == 0
