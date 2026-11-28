# Build/run environment for k1_mujoco_sim.
# The repo is bind-mounted at /workspace/NUSim by docker/k1sim.sh; this image
# only bakes the toolchain and the pinned dependencies (tools/install_deps.sh).

# The RoboCup GameController's C API, for module::MatchReferee and module::Referee. Its own stage, so the Rust
# toolchain and cargo's build tree stay out of the image. Ubuntu 22.04, as below, so the library needs no newer glibc.
FROM ubuntu:22.04 AS gamecontroller
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates curl libclang-dev python3 python3-magic python3-requests python3-tqdm \
    && rm -rf /var/lib/apt/lists/*
RUN curl -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal --default-toolchain 1.88.0
ENV PATH=/root/.cargo/bin:$PATH
# The C API is new and not in a tagged release, and its signatures have changed between commits.
ARG GC_COMMIT=21c7c6a868fe29cfdc345404859a5344b1857274
COPY docker/install-from-source.py /usr/local/bin/install-from-source
RUN install-from-source --no-toolchain --prefix /opt/gamecontroller \
    --copy game_controller_api/headers/GameController.h:include/GameController.h \
    --copy config:share/GameController/config \
    https://github.com/RoboCup-HumanoidSoccerLeague/GameController/archive/${GC_COMMIT}.tar.gz \
    -p game_controller_api

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Toolchain + the dev packages the sim links against.
# X11/GL dev packages are for GLFW (FetchContent) and MuJoCo's OpenGL renderer;
# default-jre is for fastddsgen (IDL code generation).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    git \
    ca-certificates \
    curl \
    default-jre-headless \
    libyaml-cpp-dev \
    libboost-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libxkbcommon-dev \
    libwayland-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo python (contract/model tests) — matches the C++ MuJoCo pin.
RUN pip3 install --no-cache-dir mujoco==3.10.0

# Pinned deps (MuJoCo prebuilt, Fast-DDS stack, fastddsgen) via the same script
# used for native fallback builds — single source of truth for versions.
COPY tools/install_deps.sh /tmp/install_deps.sh
RUN DEPS=/tmp/k1sim-deps-src PREFIX=/opt/k1sim-deps SRC=/tmp/k1sim-deps-src/src \
    bash /tmp/install_deps.sh --with-fastddsgen \
    && rm -rf /tmp/k1sim-deps-src /tmp/install_deps.sh

# The MuJoCo release tarball extracts its top dir as root-only (0700); the sim
# builds as the invoking --user (non-root, see docker/k1sim.sh), which then can't
# traverse it and cmake's find_library(mujoco) fails "not found". Make the deps
# tree world-readable. Own layer so it doesn't bust the slow deps build above.
RUN chmod -R a+rX /opt/k1sim-deps

# Kept as a separate layer so changes here don't invalidate the deps build above.
# cmake-curses-gui provides ccmake for `./b configure -i` (role toggle TUI).
RUN apt-get update && apt-get install -y --no-install-recommends pkg-config cmake-curses-gui \
    && rm -rf /var/lib/apt/lists/*

# libgame_controller_api.so, GameController.h, and the competitions and teams in share/GameController/config.
COPY --from=gamecontroller /opt/gamecontroller /usr/local
RUN ldconfig

ENV K1SIM_DEPS_PREFIX=/opt/k1sim-deps
WORKDIR /workspace/NUSim/mujoco
