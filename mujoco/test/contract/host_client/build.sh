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

g++ -std=c++17 -O2 -Wall \
    -I "$SDK_ROOT/include" \
    -I "$SDK_ROOT/third_party/include" \
    "$HERE/sdk_client_check.cpp" \
    -L "$SDK_ROOT/lib/x86_64" \
    -L "$SDK_ROOT/third_party/lib/x86_64" \
    -Wl,-rpath,"$SDK_ROOT/lib/x86_64" \
    -Wl,-rpath,"$SDK_ROOT/third_party/lib/x86_64" \
    -lbooster_robotics_sdk -lfastrtps -lfastcdr -lfoonathan_memory-0.7.3 -lpthread \
    -o "$HERE/sdk_client_check"

echo "Built $HERE/sdk_client_check"
