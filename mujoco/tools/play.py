"""./b play [task] [overrides] — roll out a trained K1 policy in the mjlab viewer.

Pass the checkpoint via mjlab's flag; task defaults to the K1 walk task and a leading
`--flag` is fine (forwarded to mjlab's tyro CLI):

    ./b play --checkpoint-file logs/rsl_rl/k1_walk/<run>/model_<n>.pt
    ./b play Mjlab-Walk-Flat-Booster-K1 --checkpoint-file <ckpt.pt>
"""
from _util import mjlab_run

RAW_ARGS = True  # b.py hands us the untouched arg tail (see b.py)
DEFAULT_TASK = "Mjlab-Walk-Flat-Booster-K1"


def run(argv):
    if argv and argv[0] in ("-h", "--help"):
        print(__doc__.strip())
        return
    if argv and not argv[0].startswith("-"):
        task, rest = argv[0], argv[1:]
    else:
        task, rest = DEFAULT_TASK, argv
    mjlab_run("play", task, rest)
