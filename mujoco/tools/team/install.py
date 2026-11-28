"""./b team install <team> — build another team's player image for ./b match.

    ./b team install bhuman                         from the NUSim fork of B-Human's code release on GitHub
    ./b team install bhuman --ref <branch or tag>   another branch or tag of it
    ./b team install bhuman --source ~/NUSim-B-human   a local checkout, uncommitted changes included
    ./b team install htwk                           HTWK, from the NUSim fork of its K1 code

The teams are config/match.yaml's (teams:, those with an install name). Each fork carries its own nusim/Dockerfile, built with the fork as the context: this
checks the fork out from the team's repository (or copies --source) under <build dir>/teams/<team> and builds the image
./b match runs for that team's robots.
"""

import os
import shutil
import subprocess
import sys

from _util import build_dir, config_section

import b

# config/match.yaml's teams: that have a fork to build, by their install name
TEAMS = {
    team["install"]: dict(team, codebase=name)
    for name, team in config_section("match.yaml", "teams").items()
    if "install" in team
}

# Not needed to build, and large: build output, and git history.
SKIP = shutil.ignore_patterns(".git", "Build", "build-*")


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
        help="build with AddressSanitizer, if the team's Dockerfile supports it, as <image>-asan (./b match uses it with "
        "K1SIM_<TEAM>_IMAGE=<image>-asan)",
    )


def run(team, source, repo, ref, no_cache, asan, **kwargs):
    info = dict(TEAMS[team])
    if asan:
        info["image"] += "-asan"
    mujoco = b.mujoco_dir
    staging = f"{mujoco}/{build_dir()}/teams/{team}"
    shutil.rmtree(staging, ignore_errors=True)
    os.makedirs(staging)

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

    dockerfile = f"{src}/nusim/Dockerfile"
    if not os.path.exists(dockerfile):
        sys.exit(f"error: {source or info['repo']} has no nusim/Dockerfile")
    command = ["docker", "build", "--tag", info["image"], "--file", dockerfile, *(["--no-cache"] if no_cache else [])]
    if asan:
        command += ["--build-arg", "SANITIZE_FLAGS=-fsanitize=address -fno-omit-frame-pointer -g1"]
        command += ["--build-arg", "DEBUG_TOOLS=1"]  # gdb, for K1SIM_TEAM_GDB
    command.append(src)
    if subprocess.run(command).returncode != 0:
        sys.exit(f"error: building {info['image']} failed")
    shutil.rmtree(src)
    print(f"Built {info['image']}. Play it with ./b match --away-codebase {info['codebase']}", flush=True)
