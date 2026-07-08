#!/usr/bin/env bash
# Runs the full k1_mujoco_sim test suite:
#   1. C++ unit tests (ctest, headless, inside the docker toolchain image)
#   2. Model/scene checks (host python + mujoco)
#   3. SDK wire-compat contract test (real Booster SDK client vs the sim over DDS)
# Usage: test/run_all.sh   (from anywhere; builds first if needed)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # mujoco/
BUILD_DIR="${K1SIM_BUILD_DIR:-build-docker}"

echo "=== 1/3 build + unit tests (docker, $BUILD_DIR) ==="
"$HERE/docker/k1sim.sh" build
docker run --rm -v "$(dirname "$HERE"):/workspace/NUWebots_K1" -w /workspace/NUWebots_K1/mujoco \
    --user "$(id -u):$(id -g)" "${K1SIM_IMAGE:-k1sim:latest}" \
    bash -c "ctest --test-dir $BUILD_DIR --output-on-failure"

echo "=== 2/3 model checks (host python) ==="
python3 "$HERE/test/contract/check_model.py"

echo "=== 3/3 SDK contract test ==="
python3 "$HERE/test/contract/test_sdk_roundtrip.py"

echo "=== all suites passed ==="
