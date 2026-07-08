"""./b build [targets] — build the sim (and/or specific role targets) in docker."""
from _util import k1sim


def register(command):
    command.description = "Build the sim in docker"
    command.add_argument("targets", nargs="*", help="specific ninja targets (e.g. sim-soccer); default builds all")


def run(targets, **kwargs):
    k1sim("build", *targets)
