"""K1 walk task registration (mirrors mjlab/tasks/velocity/config/g1/__init__.py,
V1 -- verified against installed mjlab.tasks.registry.register_mjlab_task)."""

from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfg import make_k1_walk_env_cfg
from .rl_cfg import k1_walk_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Walk-Flat-Booster-K1",
  env_cfg=make_k1_walk_env_cfg(),
  play_env_cfg=make_k1_walk_env_cfg(play=True),
  rl_cfg=k1_walk_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
