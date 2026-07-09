"""mjlab-based training for NUSim's K1 locomotion policy (PORT_DECISIONS.md D9).

Importing `training_mjlab.tasks.k1_walk` registers the
"Mjlab-Walk-Flat-Booster-K1" task with `mjlab.tasks.registry`.

Import this package as bare `training_mjlab`, not `mujoco.training_mjlab`: `mujoco`
is already a top-level installed package (the MuJoCo Python bindings), so this
repo's `mujoco/` directory must NOT itself become an importable package (no
`mujoco/__init__.py`) -- that would shadow the real `mujoco` for the whole venv.
Put `NUSim/mujoco/` (this package's parent, not `NUSim/` itself) on `sys.path` /
`PYTHONPATH` to import `training_mjlab.*`.
"""
