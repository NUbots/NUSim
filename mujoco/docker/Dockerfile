# Build/run environment for k1_mujoco_sim.
# The repo is bind-mounted at /workspace/NUWebots_K1 by docker/k1sim.sh; this image
# only bakes the toolchain and the pinned dependencies (tools/install_deps.sh).
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
RUN pip3 install --no-cache-dir mujoco==3.3.6

# Pinned deps (MuJoCo prebuilt, Fast-DDS stack, fastddsgen) via the same script
# used for native fallback builds — single source of truth for versions.
COPY tools/install_deps.sh /tmp/install_deps.sh
RUN DEPS=/tmp/k1sim-deps-src PREFIX=/opt/k1sim-deps SRC=/tmp/k1sim-deps-src/src \
    bash /tmp/install_deps.sh --with-fastddsgen \
    && rm -rf /tmp/k1sim-deps-src /tmp/install_deps.sh

# Kept as a separate layer so changes here don't invalidate the deps build above.
# cmake-curses-gui provides ccmake for `./b configure -i` (role toggle TUI).
RUN apt-get update && apt-get install -y --no-install-recommends pkg-config cmake-curses-gui \
    && rm -rf /var/lib/apt/lists/*

ENV K1SIM_DEPS_PREFIX=/opt/k1sim-deps
WORKDIR /workspace/NUWebots_K1/mujoco
