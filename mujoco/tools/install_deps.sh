#!/usr/bin/env bash
# Installs the k1_mujoco_sim build prerequisites that CMake does not fetch itself:
#   - MuJoCo prebuilt release (headers + libmujoco.so)
#   - Fast-CDR / foonathan_memory_vendor / Fast-DDS (built from source)
#   - Fast-DDS-Gen (fastddsgen), the IDL code generator
#
# fastddsgen is part of the standard set, not an extra: CMake now runs it at
# build time to generate the Fast-DDS types into the build dir, so they are no
# longer committed to the repo. A Java 11+ runtime is therefore a hard
# requirement of this script and of the sim build, not an optional add-on.
#
# This is normally run INSIDE the docker image build (docker/Dockerfile) with
# PREFIX=/opt/k1sim-deps — see docker/k1sim.sh for the supported workflow.
# Running it directly on a host is a fallback for native development only.
#
# Usage:
#   tools/install_deps.sh                  # installs everything, fastddsgen included
#   tools/install_deps.sh --with-fastddsgen  # accepted for backwards compatibility; does nothing
#
# (Unknown arguments are ignored rather than fatal, so older callers such as
# docker/Dockerfile that still pass --with-fastddsgen keep working unchanged.)
#
# Version pins (rationale in docs/K1_MUJOCO_SETUP.md):
#   Fast-DDS v2.13.6 matches the minor version bundled with Booster's SDK (2.13.1),
#   eliminating DDS interop drift against NUbots_K1's B1LocoClient.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # mujoco/
DEPS="${DEPS:-$HERE/.deps}"
PREFIX="${PREFIX:-$DEPS/install}"
SRC="${SRC:-$DEPS/src}"
JOBS="${JOBS:-$(nproc)}"

MUJOCO_VERSION=3.10.0
FOONATHAN_TAG=v1.3.1
FASTCDR_TAG=v2.1.3
FASTDDS_TAG=v2.13.6
FASTDDSGEN_TAG=v3.2.1

mkdir -p "$PREFIX" "$SRC"

# --- MuJoCo prebuilt ---------------------------------------------------------
if [ ! -f "$PREFIX/mujoco-$MUJOCO_VERSION/lib/libmujoco.so" ]; then
    echo "== MuJoCo $MUJOCO_VERSION (prebuilt)"
    curl -fL "https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz" \
        -o "$SRC/mujoco.tar.gz"
    tar -xzf "$SRC/mujoco.tar.gz" -C "$PREFIX"
else
    echo "== MuJoCo $MUJOCO_VERSION (cached)"
fi

# --- CMake source builds -----------------------------------------------------
build() { # build <name> <repo> <tag> [cmake args...]
    local name=$1 repo=$2 tag=$3
    shift 3
    if [ -f "$PREFIX/.stamp-$name-$tag" ]; then
        echo "== $name $tag (cached)"
        return
    fi
    echo "== $name $tag"
    rm -rf "$SRC/$name"
    git clone --depth 1 --branch "$tag" "$repo" "$SRC/$name"
    cmake -S "$SRC/$name" -B "$SRC/$name/build" -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX" \
        "$@"
    cmake --build "$SRC/$name/build" -j"$JOBS"
    cmake --install "$SRC/$name/build"
    touch "$PREFIX/.stamp-$name-$tag"
}

build foonathan_memory_vendor https://github.com/eProsima/foonathan_memory_vendor.git "$FOONATHAN_TAG"
build fastcdr https://github.com/eProsima/Fast-CDR.git "$FASTCDR_TAG" -DBUILD_SHARED_LIBS=ON
build fastdds https://github.com/eProsima/Fast-DDS.git "$FASTDDS_TAG" \
    -DBUILD_SHARED_LIBS=ON \
    -DTHIRDPARTY_Asio=FORCE \
    -DTHIRDPARTY_TinyXML2=FORCE \
    -DCOMPILE_EXAMPLES=OFF \
    -DCOMPILE_TOOLS=OFF \
    -DSECURITY=OFF

# --- fastddsgen (required, CMake generates the DDS types at build time) ------
if [ ! -f "$PREFIX/share/fastddsgen/java/fastddsgen.jar" ]; then
    echo "== Fast-DDS-Gen $FASTDDSGEN_TAG"
    rm -rf "$SRC/fastddsgen"
    git clone --recursive --depth 1 --branch "$FASTDDSGEN_TAG" \
        https://github.com/eProsima/Fast-DDS-Gen.git "$SRC/fastddsgen"
    (cd "$SRC/fastddsgen" && ./gradlew assemble)
    mkdir -p "$PREFIX/share/fastddsgen/java" "$PREFIX/bin"
    cp "$SRC/fastddsgen/share/fastddsgen/java/fastddsgen.jar" "$PREFIX/share/fastddsgen/java/"
    cp "$SRC/fastddsgen/scripts/fastddsgen" "$PREFIX/bin/"
    chmod +x "$PREFIX/bin/fastddsgen"
else
    echo "== Fast-DDS-Gen (cached)"
fi

echo
echo "Dependencies installed to $PREFIX"
echo "Configure the sim with:  cmake -S \"$HERE\" -B \"$HERE/build\" -GNinja"
