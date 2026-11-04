#!/usr/bin/env bash
# Stages the RoboCup GameController C API in mujoco/.deps/game_controller_api for module::MatchReferee
# (the sim/match role), using GameController's own dist/mkdist. .deps/ is inside the docker bind mount,
# so the image doesn't change. Run this on the host, then ./b configure. Without it the sim/match role
# is skipped and everything else builds as normal.
#
# Usage: tools/install_game_controller_api.sh [path to a GameController checkout]
#        (default: a GameController directory next to NUSim)
set -euo pipefail

# The C API is new and not in a tagged release, and its signatures have changed between commits.
GC_COMMIT=21c7c6a868fe29cfdc345404859a5344b1857274

MUJOCO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GC_DIR="$(cd "${1:-$MUJOCO_DIR/../../GameController}" && pwd)"
PREFIX="$MUJOCO_DIR/.deps/game_controller_api"

if [ "$(git -C "$GC_DIR" rev-parse HEAD)" != "$GC_COMMIT" ]; then
    echo "warning: $GC_DIR isn't at $GC_COMMIT, the commit MatchReferee was tested with" >&2
fi

rm -rf "$GC_DIR"/dist/game_controller_api-*/
(cd "$GC_DIR" && dist/mkdist --stage-only --only api)

rm -rf "$PREFIX"
mkdir -p "$(dirname "$PREFIX")"
cp -r "$GC_DIR"/dist/game_controller_api-*/ "$PREFIX"
echo "GameController API staged in $PREFIX"
