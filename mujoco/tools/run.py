"""./b run <role> [args] — run a role binary in docker (X11 + GPU + DDS passthrough).

Role is given in path form, e.g. `./b run sim/soccer` → bin/sim/soccer. Everything
after the role (including flags like --headless / --model / --rtf) passes straight
through to the binary — argparse.REMAINDER, so ./b doesn't try to parse them.
"""
import argparse

from _util import k1sim


def register(command):
    command.description = "Run a sim role binary in docker"
    command.add_argument("role", help="role in path form, e.g. sim/soccer")
    command.add_argument("args", nargs=argparse.REMAINDER, help="arguments forwarded to the role binary")


def run(role, args, **kwargs):
    k1sim("run", f"bin/{role}", *args)
