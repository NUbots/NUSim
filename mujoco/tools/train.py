"""./b train [task] [overrides] — train a K1 locomotion policy with mjlab (rsl_rl + MuJoCo Warp).

Defaults to the K1 walk task. A leading `--flag` (with no task named) is fine — everything
is forwarded to mjlab's tyro CLI, e.g.:

    ./b train                                   # walk task, default settings
    ./b train --env.scene.num-envs 4096         # override, default task
    ./b train Mjlab-Walk-Flat-Booster-K1 --agent.max-iterations 5000

Auto-exports an ONNX (obs[1,47] → actions[1,12]) on checkpoint save, drop-in for the
C++ PolicyBackend. Runs on the host uv venv — `uv sync --extra train` first.
"""
from _util import mjlab_run

RAW_ARGS = True  # b.py hands us the untouched arg tail (see b.py)
DEFAULT_TASK = "Mjlab-Walk-Flat-Booster-K1"


def run(argv):
    if argv and argv[0] in ("-h", "--help"):
        print(__doc__.strip())
        return
    # First token is the task id unless it's a flag; otherwise use the default.
    if argv and not argv[0].startswith("-"):
        task, rest = argv[0], argv[1:]
    else:
        task, rest = DEFAULT_TASK, argv
    mjlab_run("train", task, rest)
