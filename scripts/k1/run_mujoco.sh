#!/usr/bin/env bash
#
# Wrapper over mujoco/docker/k1sim.sh that brings up the MuJoCo K1 sim (GLFW viewer + FastDDS
# on domain 0). See docs/K1_MUJOCO_SETUP.md.
#
#   scripts/k1/run_mujoco.sh                    # viewer window, default field + RTF
#   K1_HEADLESS=1 scripts/k1/run_mujoco.sh       # no viewer window (--headless)
#   K1_FIELD=kidsize scripts/k1/run_mujoco.sh    # --field (middle, kidsize, no-field)
#   K1_RTF=0 scripts/k1/run_mujoco.sh            # free-run (e.g. training)
#   scripts/k1/run_mujoco.sh --config-dir path   # extra args forwarded as-is
#
# K1SIM_IMAGE overrides the docker image tag (default k1sim:latest).
# k1sim.sh's `run` builds the image and binary if they are missing.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
k1sim="$repo_root/mujoco/docker/k1sim.sh"

export K1SIM_IMAGE="${K1SIM_IMAGE:-k1sim:latest}"

args=()
[ "${K1_HEADLESS:-0}" = 1 ] && args+=(--headless)
[ -n "${K1_FIELD:-}" ] && args+=(--field "$K1_FIELD")
[ -n "${K1_RTF:-}" ] && args+=(--rtf "$K1_RTF")

exec "$k1sim" run "${args[@]}" "$@"
