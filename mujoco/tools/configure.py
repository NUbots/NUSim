"""./b configure — CMake-configure the sim in docker.

  ./b configure                         # plain configure
  ./b configure --clean                 # wipe the build dir first, then reconfigure
  ./b configure -i                      # ccmake TUI: toggle roles / options interactively
  ./b configure --unset-role sim/soccer # disable a role (repeatable)
  ./b configure --set-role sim/training # enable a role  (repeatable)

Roles are named in path form (sim/soccer) or target form (sim-soccer). After
toggling, `./b build` only builds the enabled roles.
"""
import os

from _util import k1sim


def register(command):
    command.description = "CMake-configure the sim in docker"
    command.add_argument("-i", "--interactive", action="store_true",
                         help="open the ccmake TUI to toggle roles/options")
    command.add_argument("--clean", action="store_true",
                         help="wipe the build dir (CMakeCache etc.) before configuring")
    command.add_argument("--set-role", action="append", default=[], metavar="ROLE",
                         help="enable a role, e.g. sim/soccer (repeatable)")
    command.add_argument("--unset-role", action="append", default=[], metavar="ROLE",
                         help="disable a role (repeatable)")


def _role_var(role):
    return "ROLE_" + role.strip("/").replace("/", "-")


def run(interactive, set_role, unset_role, clean, **kwargs):
    if clean:
        k1sim("clean")
    if interactive:
        k1sim("ccmake")
        return
    defs = [f"-D{_role_var(r)}=ON" for r in set_role] + [f"-D{_role_var(r)}=OFF" for r in unset_role]
    env = None
    if defs:
        env = {"K1SIM_CMAKE_ARGS": (os.environ.get("K1SIM_CMAKE_ARGS", "") + " " + " ".join(defs)).strip()}
    k1sim("configure", env=env)
