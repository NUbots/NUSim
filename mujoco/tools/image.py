"""./b image — (re)build the docker toolchain image (runs tools/install_deps.sh).

Unconditional: use this after bumping a baked dependency (e.g. MuJoCo version) —
`./b build` only builds the image when it's *missing*, so a stale image needs this.
"""
from _util import k1sim


def register(command):
    command.description = "(Re)build the docker toolchain image"


def run(**kwargs):
    k1sim("image")
