# B-Human's Booster app, built natively for x86_64 to play in NUSim (./b team install bhuman).
# The build context is staged by tools/team/install.py: src/ is the NUSim fork of B-Human's code release.
# Two stages: the toolchain and sources build the binary; the image keeps only what a player runs.

FROM ubuntu:24.04 AS build
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates ccache clang cmake libasound2-dev libclang-rt-18-dev libssl-dev lld ninja-build \
    python3 \
    && rm -rf /var/lib/apt/lists/*
COPY src /opt/bhuman
# Extra compiler and linker flags, e.g. AddressSanitizer (./b team install bhuman --asan)
ARG SANITIZE_FLAGS=""
# The build patches nothing in a checkout that already has the patches, and leaves the binary in Build/Linux/Booster
RUN CXXFLAGS="$SANITIZE_FLAGS" LDFLAGS="$SANITIZE_FLAGS" JOBS=$(nproc) /opt/bhuman/nusim/native-booster.sh

FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
# gdb only in a debug image (--asan), for K1SIM_BHUMAN_GDB
ARG DEBUG_TOOLS=""
RUN apt-get update && apt-get install -y --no-install-recommends libasound2t64 ${DEBUG_TOOLS:+gdb} \
    && rm -rf /var/lib/apt/lists/*
# The binary, its configuration and networks, and the libraries it loads (its runpath goes through Make/CMake)
COPY --from=build /opt/bhuman/Build/Linux/Booster/Develop/bhuman /opt/bhuman/Build/Linux/Booster/Develop/bhuman
COPY --from=build /opt/bhuman/Config /opt/bhuman/Config
COPY --from=build /opt/bhuman/Util/BoosterRoboticsSDK/lib/Linux /opt/bhuman/Util/BoosterRoboticsSDK/lib/Linux
COPY --from=build /opt/bhuman/Util/onnxruntime/lib/Linux /opt/bhuman/Util/onnxruntime/lib/Linux
COPY --from=build /opt/bhuman/Util/onnxruntime/LICENSE /opt/bhuman/Util/onnxruntime/LICENSE
COPY --from=build /opt/bhuman/Util/libduerwen/lib/Linux /opt/bhuman/Util/libduerwen/lib/Linux
COPY --from=build /opt/bhuman/Util/librealsense2/lib/Linux /opt/bhuman/Util/librealsense2/lib/Linux
COPY --from=build /opt/bhuman/License.md /opt/bhuman/License.md
RUN mkdir -p /opt/bhuman/Make/CMake \
    && mkdir -p /opt/nusim/home && printf 'pcm.!default { type null }\nctl.!default { type hw card 0 }\n' > /opt/nusim/home/.asoundrc \
    && chmod -R a+rwX /opt/nusim/home
# Only read by a sanitizer build: report memory errors, not the leaks at exit
ENV ASAN_OPTIONS=detect_leaks=0
WORKDIR /opt/bhuman/Build/Linux/Booster/Develop
ENTRYPOINT ["/opt/bhuman/Build/Linux/Booster/Develop/bhuman", "-c", "/opt/bhuman"]
