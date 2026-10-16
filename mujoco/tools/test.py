"""./b test — build and run the C++ unit tests with the selected backend."""

from _util import k1sim


def register(command):
    command.description = "Build and run C++ unit tests"


def run(**kwargs):
    k1sim("build")
    k1sim("test")
