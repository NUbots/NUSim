#!/usr/bin/env bash
# Regenerates mujoco/idl_gen/ from mujoco/idl/**/*.idl using the fastddsgen baked
# into the k1sim docker image (/opt/k1sim-deps/bin/fastddsgen). Run via e.g.:
#
#   docker run --rm -v $(pwd)/..:/workspace/NUSim \
#     -w /workspace/NUSim/mujoco --user $(id -u):$(id -g) \
#     k1sim:latest ./idl/regenerate.sh
#
# fastddsgen version recorded at last run: "fastddsgen version 3.2.1" (OpenJDK 11.0.31)
#
# Flags:
#   -cdr v1 -de final   Force classic CDR (no XCDR2 DELIMIT_CDR2/DHEADER framing),
#                       matching the GEN_API_VER==2 / Fast-DDS-2.13.1-era code the
#                       pinned Booster SDK ships. Verified empirically: without these
#                       flags, fastddsgen 3.2.1 defaults to `-cdr v2 -de appendable`,
#                       which emits runtime-conditional DELIMIT_CDR2 framing — a
#                       different wire format when XCDR2 representation is negotiated.
#                       See module/SdkBridge/PROTOCOL.md §5.
#   -replace            Overwrite previously generated files.
#   (deliberately NOT -typeros2) We already author the ROS2 `dds_` module nesting and
#                       trailing-underscore struct names explicitly in our .idl (to
#                       match the SDK's exact registered type names, see PROTOCOL.md
#                       §1-2). Passing -typeros2 on top of already-nested/suffixed
#                       input double-applies the transform and produces the WRONG
#                       registered name (empirically verified: `dds_::Name__` for
#                       2-level module input, `dds_::dds_::Name__` for 3-level module
#                       input — see PROTOCOL.md §2 "Empirical finding on -typeros2").
#                       Do not add this flag back without re-reading that note.
#
# Each idl/<package>/msg/*.idl generates into idl_gen/<package>/.
#
# fastddsgen also generates every file an .idl #includes, placed by that file's path
# relative to the directory fastddsgen runs in; a file outside that directory goes flat into
# the output directory. So it runs once, from a directory linking each <package>/ to
# idl/<package>/msg: an included type then generates into its own package's directory, not
# as a copy in the including package's.
set -euo pipefail

FASTDDSGEN=${FASTDDSGEN:-/opt/k1sim-deps/bin/fastddsgen}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # mujoco/
IDL_DIR="$ROOT/idl"
OUT_DIR="$ROOT/idl_gen"

echo "== fastddsgen version =="
"$FASTDDSGEN" -version

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
