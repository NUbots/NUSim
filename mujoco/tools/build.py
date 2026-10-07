"""./b build [targets] — build the sim with the selected backend."""

from _util import k1sim


def register(command):
    command.description = "Build the sim (native on macOS, Docker on Linux)"
    command.add_argument("targets", nargs="*", help="specific ninja targets (e.g. sim-soccer); default builds all")


def run(targets, **kwargs):
    k1sim("build", *targets)
