"""./b team install <team> — build another team's player image for ./b match.

    ./b team install bhuman                         from the NUSim fork of B-Human's code release on GitHub
    ./b team install bhuman --ref <branch or tag>   another branch or tag of it
    ./b team install bhuman --source ~/NUSim-B-human   a local checkout, uncommitted changes included

Each team has a directory in docker/teams/ with a Dockerfile that builds its code from src/. This stages that
directory under <build dir>/teams/<team>, with src/ checked out from the team's repository (or copied from --source),
and builds the image ./b match runs for that team's robots.
"""

import shutil
import subprocess
import sys

from _util import build_dir

import b

# What ./b match runs for each team, and where its code comes from.
TEAMS = {
    "bhuman": dict(
        image="nusim-bhuman",
        codebase="B-Human",  # its name in match.py's CODEBASES
        repo="https://github.com/jpptm/NUSim-B-human.git",
        ref="main",
    ),
}

# Not needed to build, and large: build output, and git history.
SKIP = shutil.ignore_patterns(".git", "Build")


def register(command):
    command.description = "Build another team's player image for ./b match"
    command.add_argument("team", choices=sorted(TEAMS), help="the team")
    command.add_argument("--source", help="a local checkout to build instead of the repository")
    command.add_argument("--repo", help="the repository to check out (default: the team's)")
    command.add_argument("--ref", help="the branch or tag to check out (default: the team's)")
    command.add_argument("--no-cache", action="store_true", help="rebuild every image layer")
    command.add_argument(
        "--asan",
        action="store_true",
        help="build with AddressSanitizer, as <image>-asan (./b match uses it with K1SIM_BHUMAN_IMAGE=<image>-asan)",
    )


def run(team, source, repo, ref, no_cache, asan, **kwargs):
    info = dict(TEAMS[team])
    if asan:
        info["image"] += "-asan"
    mujoco = b.mujoco_dir
    staging = f"{mujoco}/{build_dir()}/teams/{team}"
    shutil.rmtree(staging, ignore_errors=True)
    shutil.copytree(f"{mujoco}/docker/teams/{team}", staging)

    src = f"{staging}/src"
    if source:
        print(f"Copying {source}", flush=True)
        shutil.copytree(source, src, symlinks=True, ignore=SKIP)
    else:
        repo, ref = repo or info["repo"], ref or info["ref"]
        print(f"Checking out {repo} ({ref})", flush=True)
        subprocess.run(["git", "clone", "--depth", "1", "--branch", ref, repo, src], check=True)
        shutil.rmtree(f"{src}/.git")
        shutil.rmtree(f"{src}/Build", ignore_errors=True)

    command = ["docker", "build", "--tag", info["image"], *(["--no-cache"] if no_cache else [])]
    if asan:
        command += ["--build-arg", "SANITIZE_FLAGS=-fsanitize=address -fno-omit-frame-pointer -g1"]
        command += ["--build-arg", "DEBUG_TOOLS=1"]  # gdb, for K1SIM_BHUMAN_GDB
    command.append(staging)
    if subprocess.run(command).returncode != 0:
        sys.exit(f"error: building {info['image']} failed")
    shutil.rmtree(src)
    print(f"Built {info['image']}. Play it with ./b match --away-codebase {info['codebase']}", flush=True)
