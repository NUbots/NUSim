"""RL configuration for the K1 walk task: G1 shape, not ParameterWalk's raw PPO numbers.

D7: ParameterWalk's custom-PPO hyperparams (learning_rate=5e-6, entropy_coef=-0.01 with an
inverted sign convention, hand-rolled GAE/clip) don't transfer to rsl_rl's PPO
semantics -- reuse mjlab's proven G1 shape instead
(mjlab/tasks/velocity/config/g1/rl_cfg.py).
"""

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def k1_walk_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Create RL runner configuration for the K1 walk task."""
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
      distribution_cfg={
        "class_name": "GaussianDistribution",
        "init_std": 1.0,
        "std_type": "scalar",
      },
    ),
    critic=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.01,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="k1_walk",
    # TensorBoard, not mjlab's wandb default — no API key / login needed to train.
    # Override on the CLI with `--agent.logger wandb` if you want W&B.
    logger="tensorboard",
    save_interval=50,
    num_steps_per_env=24,
    max_iterations=30_000,
  )
