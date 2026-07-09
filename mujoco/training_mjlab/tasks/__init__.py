"""mjlab task configs for the K1.

Importing this package registers every task with `mjlab.tasks.registry`, so the
`./b train` / `./b play` launcher only needs `import training_mjlab.tasks`. To add a
new task (e.g. dribble, kick): create a `k1_<name>/` subpackage (mirror `k1_walk/`)
whose `__init__.py` calls `register_mjlab_task(...)`, then add one import line below.
See ../README.md "Adding a new task".
"""

from . import k1_walk  # noqa: F401 — registers Mjlab-Walk-Flat-Booster-K1
