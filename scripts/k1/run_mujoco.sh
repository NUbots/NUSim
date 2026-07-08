#!/usr/bin/env bash
#
# Thin wrapper over mujoco/docker/k1sim.sh: brings up the self-contained
# MuJoCo K1 sim (GLFW viewer + FastDDS on domain 0). This replaces BOTH
# Booster's Webots build AND the `mck` runner in one step — no gated
# downloads, no separate runner process. See docs/K1_MUJOCO_SETUP.md.
#
#   scripts/k1/run_mujoco.sh                    # viewer window, default model + RTF
#   K1_HEADLESS=1 scripts/k1/run_mujoco.sh       # no viewer window (--headless)
#   K1_MODEL=mujoco/models/k1/foo.xml run_mujoco.sh
#   K1_RTF=0 scripts/k1/run_mujoco.sh            # free-run (e.g. training)
#   scripts/k1/run_mujoco.sh --config-dir path   # extra args forwarded as-is
#
# K1SIM_IMAGE overrides the docker image tag (default k1sim:latest).
# k1sim.sh's own `run` subcommand builds the image/binary automatically the
# first time (or whenever the binary is missing) — nothing to build by hand.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
k1sim="$repo_root/mujoco/docker/k1sim.sh"

export K1SIM_IMAGE="${K1SIM_IMAGE:-k1sim:latest}"

args=()
[ "${K1_HEADLESS:-0}" = 1 ] && args+=(--headless)
[ -n "${K1_MODEL:-}" ] && args+=(--model "$K1_MODEL")
[ -n "${K1_RTF:-}" ] && args+=(--rtf "$K1_RTF")

exec "$k1sim" run "${args[@]}" "$@"
