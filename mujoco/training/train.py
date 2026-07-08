#!/usr/bin/env python3
"""Train a locomotion policy with MJX + brax PPO.

    python training/train.py --policy policies/walk.py --iterations 200
    ./b train walk.py --iterations 200 --render

Loads the Policy subclass from the given script (its module-level `POLICY`),
trains with brax PPO on GPU-parallel MJX envs, prints metrics each eval, and
checkpoints params. `--render` rolls the trained/loaded policy out live in the
MuJoCo viewer. Export to ONNX for deployment with export_onnx.py.
"""
from __future__ import annotations

import argparse
import functools
import importlib.util
import os
import pickle
import time

_HERE = os.path.dirname(os.path.abspath(__file__))


def load_policy_class(script):
    path = script if os.path.isabs(script) else os.path.join(_HERE, script)
    spec = importlib.util.spec_from_file_location("_k1_policy_module", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "POLICY"):
        raise SystemExit(f"{script}: expected a module-level POLICY = <YourPolicy> class")
    return module.POLICY


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", default="policies/walk.py", help="policy script (defines POLICY)")
    ap.add_argument("--iterations", type=int, default=200, help="PPO eval iterations")
    ap.add_argument("--num-envs", type=int, default=2048)
    ap.add_argument("--timesteps", type=int, default=0, help="override total env steps (0 = iterations*batch)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--checkpoint", default="checkpoints/walk.pkl")
    ap.add_argument("--render", action="store_true", help="roll out the trained policy in the viewer")
    ap.add_argument("--load", default="", help="load params from a checkpoint before training/render")
    args = ap.parse_args()

    # Imported lazily so --help works without jax/brax installed.
    import jax
    from brax.training.agents.ppo import networks as ppo_networks
    from brax.training.agents.ppo import train as ppo

    Policy = load_policy_class(args.policy)
    env = Policy()
    print(f"[train] policy={args.policy} obs={env.observation_size} act={env.action_size} "
          f"envs={args.num_envs} device={jax.devices()[0].platform}")

    hidden = (256, 128, 128)
    network_factory = functools.partial(ppo_networks.make_ppo_networks, policy_hidden_layer_sizes=hidden)

    total = args.timesteps or (args.iterations * args.num_envs * env.episode_length)
    t0 = time.time()

    # Scale the minibatch/batch structure to num_envs so small runs (CPU smoke)
    # and large GPU runs both satisfy brax's divisibility constraints.
    num_minibatches = max(1, min(32, args.num_envs // 2))
    batch_size = max(1, min(1024, args.num_envs // 2))

    def progress(step, metrics):
        r = metrics.get("eval/episode_reward", float("nan"))
        rl = metrics.get("eval/episode_reward_std", float("nan"))
        xv = metrics.get("eval/episode_x_vel", float("nan"))
        print(f"[train] step={step:>10}  reward={r:8.3f} ± {rl:6.3f}  x_vel={xv:6.3f}  "
              f"({time.time() - t0:5.0f}s)")

    make_inference_fn, params, _ = ppo.train(
        environment=env,
        num_timesteps=total,
        num_evals=max(1, args.iterations),
        episode_length=env.episode_length,
        num_envs=args.num_envs,
        batch_size=batch_size,
        unroll_length=20,
        num_minibatches=num_minibatches,
        num_updates_per_batch=4,
        learning_rate=3e-4,
        entropy_cost=1e-2,
        discounting=0.97,
        network_factory=network_factory,
        seed=args.seed,
        progress_fn=progress,
    )

    ckpt = args.checkpoint if os.path.isabs(args.checkpoint) else os.path.join(_HERE, args.checkpoint)
    os.makedirs(os.path.dirname(ckpt), exist_ok=True)
    with open(ckpt, "wb") as f:
        pickle.dump({"params": params, "hidden": hidden}, f)
    print(f"[train] saved checkpoint → {ckpt}")

    if args.render:
        render_rollout(env, make_inference_fn, params)


def render_rollout(env, make_inference_fn, params, seconds=20.0):
    """Live viewer rollout of the policy (the 'show me the policy' option)."""
    import jax
    import mujoco
    import mujoco.viewer
    import numpy as np
    from mujoco import mjx

    inference = jax.jit(make_inference_fn(params, deterministic=True))
    rng = jax.random.PRNGKey(0)
    state = jax.jit(env.reset)(rng)
    step = jax.jit(env.step)

    m, d = env.mj_model, mujoco.MjData(env.mj_model)
    print("[render] launching viewer — Ctrl-C to stop")
    with mujoco.viewer.launch_passive(m, d) as viewer:
        t_end = time.time() + seconds
        while viewer.is_running() and time.time() < t_end:
            act, _ = inference(state.obs, rng)
            state = step(state, act)
            # mirror MJX qpos/qvel into the viewer's MjData
            d.qpos[:] = np.asarray(state.pipeline_state.qpos)
            d.qvel[:] = np.asarray(state.pipeline_state.qvel)
            mujoco.mj_forward(m, d)
            viewer.sync()
            time.sleep(env.dt)


if __name__ == "__main__":
    main()
