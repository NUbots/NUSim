#!/usr/bin/env bash
# Docker workflow for k1_mujoco_sim (mirrors the NUbots_K1 "./b" philosophy:
# the toolchain lives in an image, the repo is bind-mounted, artifacts stay in
# the repo's build-docker/ directory).
#
#   docker/k1sim.sh image             build/refresh the toolchain image
#   docker/k1sim.sh configure         cmake configure only (inside the container)
#   docker/k1sim.sh build [targets]   ninja (configures first if needed)
#   docker/k1sim.sh clean             wipe the build dir (force a fresh configure)
#   docker/k1sim.sh test              ctest (headless, inside the container)
#   docker/k1sim.sh run <bin> [args]  run a role binary (X11 + GPU passthrough); default bin/sim/soccer
#   docker/k1sim.sh shell             interactive shell in the build environment
#
# DDS: the sim runs with --network host --ipc host so FastDDS (SHM + UDP,
# domain 0) reaches NUbots_K1 containers exactly like the old mck-on-host setup.
# GPU: uses --gpus all when the NVIDIA container toolkit is installed, otherwise
# falls back to /dev/dri (mesa). See docs/K1_MUJOCO_SETUP.md.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" # NUSim/
IMAGE="${K1SIM_IMAGE:-k1sim:latest}"
BUILD_DIR="${K1SIM_BUILD_DIR:-build-docker}"
CMAKE_ARGS=${K1SIM_CMAKE_ARGS:-}

image() {
    docker build -t "$IMAGE" -f "$REPO/mujoco/docker/Dockerfile" "$REPO/mujoco"
}

ensure_image() {
    docker image inspect "$IMAGE" > /dev/null 2>&1 || image
}

# Common flags for every container invocation.
common_flags() {
    echo -n "--rm -v $REPO:/workspace/NUSim -w /workspace/NUSim/mujoco"
    # Run as the invoking user so build artifacts in the bind mount aren't root-owned
    echo -n " --user $(id -u):$(id -g)"
}

# Extra flags for interactive/GUI/DDS runs.
run_flags() {
    echo -n " --network host --ipc host"
    if [ -n "${DISPLAY:-}" ]; then
        echo -n " -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
    fi
    # Mesa GL via the DRM nodes (works for iGPUs, and is a harmless no-op otherwise).
    if [ -d /dev/dri ]; then
        echo -n " --device /dev/dri"
    fi
    # NVIDIA: prefer explicit CDI device naming over `--gpus all` — on Docker 25+
    # a bare --gpus resolves vendor-by-vendor through CDI and hard-fails on
    # hybrid-graphics machines with "AMD CDI spec not found" even though the
    # NVIDIA spec exists. Fall back to the classic nvidia runtime if configured.
    if ls /etc/cdi/nvidia*.yaml /var/run/cdi/nvidia*.yaml > /dev/null 2>&1; then
        echo -n " --device nvidia.com/gpu=all -e NVIDIA_DRIVER_CAPABILITIES=all"
    elif docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
        echo -n " --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all"
    fi
}

build() {
    ensure_image
    # shellcheck disable=SC2046
    docker run $(common_flags) "$IMAGE" bash -c "
        cmake -S . -B $BUILD_DIR -GNinja $CMAKE_ARGS &&
        cmake --build $BUILD_DIR -j\$(nproc)"
}

configure() {
    ensure_image
    # shellcheck disable=SC2046
    docker run $(common_flags) "$IMAGE" bash -c "cmake -S . -B $BUILD_DIR -GNinja $CMAKE_ARGS"
}

clean() {
    # Build artifacts are user-owned (see --user in common_flags), so a host-side
    # rm is enough — no container needed. Removes the whole dir incl. CMakeCache.txt.
    rm -rf "${REPO:?}/mujoco/$BUILD_DIR"
    echo "removed mujoco/$BUILD_DIR"
}

case "${1:-}" in
    image) image ;;
    clean) clean ;;
    configure) configure ;;
    ccmake)
        ensure_image
        [ -f "$REPO/mujoco/$BUILD_DIR/build.ninja" ] || configure
        # shellcheck disable=SC2046
        docker run -it $(common_flags) "$IMAGE" bash -c "ccmake -S . -B $BUILD_DIR"
        ;;
    build)
        shift
        ensure_image
        [ -f "$REPO/mujoco/$BUILD_DIR/build.ninja" ] || configure
        target_arg=""
        [ $# -gt 0 ] && target_arg="--target $*"
        # shellcheck disable=SC2046
        docker run $(common_flags) "$IMAGE" bash -c "cmake --build $BUILD_DIR -j\$(nproc) $target_arg"
        ;;
    test)
        ensure_image
        # shellcheck disable=SC2046
        docker run $(common_flags) "$IMAGE" bash -c "ctest --test-dir $BUILD_DIR --output-on-failure"
        ;;
    run)
        # run <bin-relpath> [args...]   e.g. run bin/sim/soccer --headless
        shift
        bin="${1:-bin/sim/soccer}"
        [ $# -gt 0 ] && shift
        ensure_image
        [ -x "$REPO/mujoco/$BUILD_DIR/$bin" ] || build
        [ -n "${DISPLAY:-}" ] && xhost +local: > /dev/null 2>&1 || true
        # shellcheck disable=SC2046
        docker run -it $(common_flags) $(run_flags) "$IMAGE" "./$BUILD_DIR/$bin" "$@"
        ;;
    shell)
        ensure_image
        # shellcheck disable=SC2046
        docker run -it $(common_flags) $(run_flags) "$IMAGE" bash
        ;;
    *)
        grep '^#   docker' "$0" | sed 's/^# *//'
        exit 1
        ;;
esac
