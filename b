#!/usr/bin/env bash
# NUbots-style command dispatcher for the K1 MuJoCo sim.
exec python3 "$(dirname "$0")/mujoco/b.py" "$@"
