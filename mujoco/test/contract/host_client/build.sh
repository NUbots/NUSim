#!/usr/bin/env bash
# Builds sdk_client_check ON THE HOST (not in the k1sim docker image — the SDK's
# bundled Fast-DDS/fastcdr third_party libs + g++ 11 are what the host has, and
# this client is deliberately never linked into the sim). See
# test/contract/README.md for the full rationale.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="${BOOSTER_SDK_ROOT:-/home/nubots/Workspace/booster/booster_robotics_sdk}"

if [ ! -f "$SDK_ROOT/lib/x86_64/libbooster_robotics_sdk.a" ]; then
    echo "error: BOOSTER_SDK_ROOT ($SDK_ROOT) doesn't look like a built booster_robotics_sdk checkout" >&2
    exit 1
fi

# Older SDKs (7fb7287, 324946e7) link against their bundled third_party Fast-DDS; newer ones
# (d5d8f7ae, what NUbots_K1 main pins) carry a renamed copy inside the .a itself.
if [ -d "$SDK_ROOT/third_party/lib/x86_64" ]; then
    DDS_FLAGS=(-I "$SDK_ROOT/third_party/include"
        -L "$SDK_ROOT/third_party/lib/x86_64"
        -Wl,-rpath,"$SDK_ROOT/third_party/lib/x86_64")
    DDS_LIBS=(-lfastrtps -lfastcdr -lfoonathan_memory-0.7.3)
else
    DDS_FLAGS=()
    DDS_LIBS=(-ldl -lrt)
fi

g++ -std=c++17 -O2 -Wall \
    -I "$SDK_ROOT/include" \
    "${DDS_FLAGS[@]}" \
    "$HERE/sdk_client_check.cpp" \
    -L "$SDK_ROOT/lib/x86_64" \
    -Wl,-rpath,"$SDK_ROOT/lib/x86_64" \
    -lbooster_robotics_sdk "${DDS_LIBS[@]}" -lpthread \
    -o "$HERE/sdk_client_check"

echo "Built $HERE/sdk_client_check"
