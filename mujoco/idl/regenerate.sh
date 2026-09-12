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
set -euo pipefail

FASTDDSGEN=${FASTDDSGEN:-/opt/k1sim-deps/bin/fastddsgen}
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" # mujoco/
IDL_DIR="$ROOT/idl"
OUT_DIR="$ROOT/idl_gen"

echo "== fastddsgen version =="
"$FASTDDSGEN" -version

PACKAGES="booster_interface booster_msgs builtin_interfaces std_msgs geometry_msgs nav_msgs"
for pkg in $PACKAGES; do
    rm -rf "$OUT_DIR/$pkg"
    mkdir -p "$OUT_DIR/$pkg"
done

# Every package's msg/ directory is on the include path, so an .idl can #include a type from
# another package by file name (nav_msgs/Odometry.idl includes std_msgs' Header.idl).
INCLUDES=()
for pkg in $PACKAGES; do
    INCLUDES+=(-I "$IDL_DIR/$pkg/msg")
done

gen() {
    local src_dir="$1"
    local out_dir="$2"
    shift 2
    (cd "$src_dir" && "$FASTDDSGEN" -cdr v1 -de final -replace "${INCLUDES[@]}" -d "$out_dir" "$@")
}

# booster_interface::msg — #include paths inside these .idl files are resolved
# relative to each file's own directory, so order in the arg list doesn't matter.
gen "$IDL_DIR/booster_interface/msg" "$OUT_DIR/booster_interface" \
    ImuState.idl MotorState.idl LowState.idl Odometer.idl FallDownState.idl \
    BatteryState.idl ButtonEvent.idl MotorCmd.idl LowCmd.idl

# booster_msgs::msg
gen "$IDL_DIR/booster_msgs/msg" "$OUT_DIR/booster_msgs" \
    RpcReqMsg.idl RpcRespMsg.idl

# geometry_msgs::msg (rt/head_pose, and the pose/twist halves of rt/odom)
gen "$IDL_DIR/geometry_msgs/msg" "$OUT_DIR/geometry_msgs" \
    Point.idl Quaternion.idl Pose.idl PoseWithCovariance.idl Vector3.idl Twist.idl \
    TwistWithCovariance.idl

# builtin_interfaces::msg, std_msgs::msg, nav_msgs::msg (rt/odom)
gen "$IDL_DIR/builtin_interfaces/msg" "$OUT_DIR/builtin_interfaces" Time.idl
gen "$IDL_DIR/std_msgs/msg" "$OUT_DIR/std_msgs" Header.idl
gen "$IDL_DIR/nav_msgs/msg" "$OUT_DIR/nav_msgs" Odometry.idl

echo "== Generated into $OUT_DIR. Registered type names: =="
grep -rho 'setName("[^"]*")' "$OUT_DIR" | sort -u
