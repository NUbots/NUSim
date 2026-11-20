"""./b skills [args] — play one K1 from the keyboard with the mjlab policies (role sim/skills).
./b skills sync [--mjlab DIR] — copy mjlab's current best policies into mujoco/policies/skills: the files
policies/BENCHMARKS.md's "use this one" table names, and for a skill it doesn't name (the walk) the spec's.

Keys, in the sim window: W = walk to a ball / dribble, Q = pass at the robot, E = ball at its feet,
W/A/S/D = dribble (Shift faster), Space = hold to charge a shot and release to kick, R = stand the
robot back up, P = pause. The viewer's Training buttons switch between free play and the rondo
(three robots passing round a fourth); four robots are spawned for it unless --robots is given.
Settings are in mujoco/config/skills.yaml.
"""

import argparse
import glob
import os
import re
import shutil
import time

from _util import k1sim

import b

DEST = os.path.join(b.mujoco_dir, "policies", "skills")
SPEC = "policies/CONTROLLER_INTERFACE.md"
BENCHMARKS = "policies/BENCHMARKS.md"

# Our file name -> how BENCHMARKS.md's "| Skill | File |" table names that skill.
BEST = {
    "receive": "Receive",
    "dribble": "Dribble",
    "kick": "Kick (ground)",
    "kick_air": "Air shot",
    "first_touch": "Directional first touch",
    "first_pass": "First-time pass",
}

# Our file name -> how the spec's policy table names that skill (the start of its first cell).
SKILLS = {
    "walk": "Walk",
    "receive": "Receive (standing or walking)",
    "dribble": "Dribble",
    "kick": "Kick",  # "Kick (ground)"
    "kick_air": "Air shot",
    "first_touch": "Directional first touch",
    "first_pass": "First-time pass",
}


def register(command):
    command.description = "Keyboard play with the mjlab policies, or `sync` to refresh the policies"
    command.add_argument("--mjlab", default="~/mjlab", help="mjlab checkout to sync from (default ~/mjlab)")
    command.add_argument("args", nargs=argparse.REMAINDER, help="`sync`, or arguments for the sim (e.g. --field)")


def _spec_files(mjlab):
    """Skill -> candidate .onnx paths (globs allowed), read from the spec's policy table.

    A skill's candidates are the .onnx paths in its table row. The kick's also include any named in
    the spec's "Kick note" (where the finished run will be), so sync picks that up once it exists.
    """
    with open(os.path.join(mjlab, SPEC)) as file:
        text = file.read()
    found = {}
    for line in text.splitlines():
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 2:
            continue
        for name, label in SKILLS.items():
            if name not in found and cells[0].startswith(label):
                found[name] = re.findall(r"`([^`]+\.onnx)`", cells[1])
    note = re.search(r"\*\*Kick note:\*\*(.*?)\n\n", text, re.S)
    if note:
        found.setdefault("kick", []).extend(re.findall(r"`([^`]+\.onnx)`", note.group(1)))
    return found


def _best_files(policies):
    """Skill -> the .onnx path BENCHMARKS.md's first "| Skill | File |" table picks, where it resolves.

    A cell's first `...` names a checkpoint (`folder/model_<n>`, exported as model_<n>.onnx next to
    it) or a folder with one top-level export. A checkpoint without its own export falls back to the
    folder's only top-level .onnx when the folder holds that checkpoint and no other (the run's
    final export). Anything else is left to the spec.
    """
    try:
        with open(os.path.join(os.path.dirname(policies), BENCHMARKS)) as file:
            lines = file.read().splitlines()
    except OSError:
        return {}
    found, in_table = {}, False
    for line in lines:
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if cells[:2] == ["Skill", "File"]:
            in_table = True
            continue
        if in_table and not line.startswith("|"):
            break
        if not in_table or len(cells) < 2:
            continue
        name = next((n for n, label in BEST.items() if cells[0] == label), None)
        named = re.findall(r"`([^`]+)`", cells[1])
        if not name or not named:
            continue
        target = os.path.join(policies, named[0].removesuffix(".onnx"))
        folder = target if os.path.isdir(target) else os.path.dirname(target)
        exports = glob.glob(os.path.join(folder, "*.onnx"))
        if os.path.isfile(target + ".onnx"):
            found[name] = target + ".onnx"
        elif len(exports) == 1 and (
            os.path.isdir(target) or glob.glob(os.path.join(folder, "*.pt")) == [target + ".pt"]
        ):
            found[name] = exports[0]
    return found


def _sync(mjlab):
    mjlab = os.path.expanduser(mjlab)
    policies = os.path.join(mjlab, "policies")
    try:
        wanted = _spec_files(mjlab)
    except OSError as error:
        raise SystemExit(f"cannot read the spec: {error}")
    best = _best_files(policies)
    os.makedirs(DEST, exist_ok=True)
    sources = []
    for name in SKILLS:
        candidates = [best[name]] if name in best else []
        for pattern in [] if candidates else wanted.get(name, []):
            candidates += glob.glob(os.path.join(policies, pattern))
        if not candidates:
            print(f"{name:8s} the spec names {wanted.get(name) or 'nothing'}, not found under {policies}; left as it is")
            continue
        newest = max(candidates, key=os.path.getmtime)
        shutil.copy2(newest, os.path.join(DEST, name + ".onnx"))
        stamp = time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(newest)))
        print(f"{name:8s} <- {newest} ({stamp})")
        sources.append(f"{name}.onnx <- {newest} ({stamp})")
    with open(os.path.join(DEST, "SOURCES.txt"), "w") as file:
        file.write(f"Copied by ./b skills sync from {mjlab}/{BENCHMARKS} (else {SPEC})\n" + "\n".join(sources) + "\n")


def run(mjlab, args, **kwargs):
    if args[:1] == ["sync"]:
        _sync(mjlab)
    else:
        # Four robots, for the rondo (the viewer's Training buttons): in free play the other three
        # stand on the touchline. --robots 1 gives the old single-robot scene.
        if "--robots" not in args and "--game" not in args:
            args = [*args, "--robots", "4"]
        k1sim("run", "bin/sim/skills", *args)
