"""./b skills [args] — play one K1 from the keyboard with the mjlab policies (role sim/skills).
./b skills sync [--mjlab DIR] — copy the policies mjlab's controller spec names into mujoco/policies/skills.

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


def _sync(mjlab):
    mjlab = os.path.expanduser(mjlab)
    policies = os.path.join(mjlab, "policies")
    try:
        wanted = _spec_files(mjlab)
    except OSError as error:
        raise SystemExit(f"cannot read the spec: {error}")
    os.makedirs(DEST, exist_ok=True)
    sources = []
    for name in SKILLS:
        candidates = []
        for pattern in wanted.get(name, []):
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
        file.write(f"Copied by ./b skills sync from {mjlab}/{SPEC}\n" + "\n".join(sources) + "\n")


def run(mjlab, args, **kwargs):
    if args[:1] == ["sync"]:
        _sync(mjlab)
    else:
        # Four robots, for the rondo (the viewer's Training buttons): in free play the other three
        # stand on the touchline. --robots 1 gives the old single-robot scene.
        if "--robots" not in args and "--game" not in args:
            args = [*args, "--robots", "4"]
        k1sim("run", "bin/sim/skills", *args)
