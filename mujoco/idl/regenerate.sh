#!/usr/bin/env bash
# Generates the Fast-DDS type support from mujoco/idl/**/*.idl using the fastddsgen
# baked into the k1sim docker image (/opt/k1sim-deps/bin/fastddsgen).
#
# cmake/IdlGen.cmake runs this as a build step with OUT_DIR=${CMAKE_BINARY_DIR}/idl_gen; the output
# is a build artifact, never committed. To inspect the output by hand, point OUT_DIR somewhere
# scratch:
#
#   docker run --rm -v $(pwd)/..:/workspace/NUSim \
#     -w /workspace/NUSim/mujoco --user $(id -u):$(id -g) \
#     -e OUT_DIR=/tmp/idl_gen k1sim:latest ./idl/regenerate.sh
#
# NOTE: OUT_DIR is wiped (rm -rf) before generating. Never aim it at a source dir.
#
# The generator version is asserted, not just reported: the wire format depends on the version
# and flags, and the generated code never shows up in a diff. See module/SdkBridge/PROTOCOL.md.
#
# Flags:
#   -cdr v1 -de final   Classic CDR, matching the pinned Booster SDK's generated code. fastddsgen
#                       3.2.1 otherwise defaults to -cdr v2 -de appendable, a different wire format
#                       when XCDR2 is negotiated (PROTOCOL.md §5).
#   -replace            Overwrite previously generated files.
#   (not -typeros2)     The .idl files already spell out the ROS2 `dds_` nesting and trailing
#                       underscores; -typeros2 would apply them twice and register the wrong type
#                       names (PROTOCOL.md §2).
#
# Each idl/<package>/msg/*.idl generates into idl_gen/<package>/.
#
# fastddsgen also generates every #included file, placed by its path relative to where it runs (a
# file outside that directory lands flat in OUT_DIR). So it runs once, from a directory linking each
# <package>/ to idl/<package>/msg, and an included type generates into its own package's directory.
set -euo pipefail

FASTDDSGEN=${FASTDDSGEN:-/opt/k1sim-deps/bin/fastddsgen}
FASTDDSGEN_VERSION=3.2.1 # must match FASTDDSGEN_TAG in tools/install_deps.sh
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # mujoco/
IDL_DIR="$ROOT/idl"
# Default lands under build/ (gitignored), never in the source tree: a hand run must not
# be able to recreate a committed-looking idl_gen/. CMake always passes OUT_DIR explicitly.
OUT_DIR="${OUT_DIR:-$ROOT/build/idl_gen}"

# Hard-fail on a generator other than the pinned one. fastddsgen 3.2.1 with the flags
# below emits classic CDR; other versions change their defaults (see the -cdr/-de note
# above), which would silently alter the wire format the robot decodes.
version_output="$("$FASTDDSGEN" -version 2>&1 | tr -d '\r')"
echo "== $(echo "$version_output" | grep -i '^fastddsgen' || echo "$version_output")"
if ! grep -qiE "^fastddsgen version $FASTDDSGEN_VERSION\$" <<< "$version_output"; then
    echo "error: expected fastddsgen $FASTDDSGEN_VERSION, got:" >&2
    echo "$version_output" >&2
    echo "       reinstall with tools/install_deps.sh (FASTDDSGEN_TAG pins v$FASTDDSGEN_VERSION)." >&2
    exit 1
fi

# Where fastddsgen runs: <package>/ -> idl/<package>/msg
VIEW="$(mktemp -d)"
trap 'rm -rf "$VIEW"' EXIT

INCLUDES=()
FILES=()
for dir in "$IDL_DIR"/*/msg; do
    pkg="$(basename "$(dirname "$dir")")"
    ln -s "$dir" "$VIEW/$pkg"
    # So an .idl can #include another package's type by file name
    INCLUDES+=(-I "$pkg")
    for idl in "$dir"/*.idl; do
        FILES+=("$pkg/$(basename "$idl")")
    done
done

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"
(cd "$VIEW" && "$FASTDDSGEN" -cdr v1 -de final -replace "${INCLUDES[@]}" -d "$OUT_DIR" "${FILES[@]}")

# Each idl_gen/<package>/ holds only its own package's types; any other is a duplicate
for dir in "$OUT_DIR"/*/; do
    pkg="$(basename "$dir")"
    for header in "$dir"*PubSubTypes.h; do
        type="$(basename "$header" PubSubTypes.h)"
        if [ ! -f "$IDL_DIR/$pkg/msg/$type.idl" ]; then
            echo "error: idl_gen/$pkg/ has $type, which is not in idl/$pkg/msg/" >&2
            exit 1
        fi
    done
done
if [ -n "$(find "$OUT_DIR" -maxdepth 1 -type f)" ]; then
    echo "error: files generated outside a package directory in $OUT_DIR" >&2
    exit 1
fi

echo "== Generated into $OUT_DIR. Registered type names: =="
grep -rho 'setName("[^"]*")' "$OUT_DIR" | sort -u
