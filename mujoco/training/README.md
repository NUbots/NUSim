# K1 locomotion training (MJX)

GPU-parallel RL training for the K1 walk policy, in the **same MuJoCo physics** the
sim deploys (MJX → no sim2sim gap). Produces an ONNX the C++ `PolicyBackend` loads.
Framework only — you run the training (it needs a GPU and reward tuning).

## Setup

Training deps are the `train` extra in the repo-root `pyproject.toml` (managed by
[uv](https://docs.astral.sh/uv/)). From the repo root:

```bash
uv sync --extra train      # installs jax[cuda12]/mjx/brax into .venv; ./b train uses it
```

CPU-only smoke test: swap `jax[cuda12]` for plain `jax` in the `train` extra, then
re-sync. GPU needs CUDA 12 (RTX 3080 Ti).

## Train

```bash
./b train walk.py                              # = python training/train.py --policy policies/walk.py
./b train walk.py --iterations 400 --num-envs 4096
./b train walk.py --render                     # watch the policy live in the MuJoCo viewer after the training is finished
```

Metrics (episode reward, x-velocity) print each eval; params checkpoint to
`training/checkpoints/`. `--render` (the "show me the policy" option) rolls the
current policy out in `mujoco.viewer`.

## Deploy

Export the trained checkpoint to the deploy ONNX — either in one shot at the end of
training, or standalone from a saved checkpoint:

```bash
./b train walk.py --export                                              # train, then export → models/walk.onnx
python training/export_onnx.py --checkpoint checkpoints/walk.pkl --out models/walk.onnx   # standalone
```

Then point the sim at it and build with ONNX:

```yaml
# mujoco/config/locomotion.yaml
backend: policy
policy: { path: "training/models/walk.onnx", action_scale: 0.5, gait_frequency: 1.5 }
```
```bash
K1SIM_CMAKE_ARGS="-DK1_WITH_ONNX=ON" ./b build
./b run sim/soccer      # NUbots Move commands now drive the trained gait
```

## Writing a policy

Subclass `Policy` (`training/policy.py`) and override the **cost** and command
sampler; everything else — the 47-dim obs, the 12-leg action, the gait clock, the
PD sub-loop — is shared so every policy honours one deploy contract
(`OBS_ACTION_CONTRACT.md`). Minimal example — see `policies/walk.py`:

```python
from policy import Policy
class Walk(Policy):
    def sample_command(self, rng): ...          # [vx, vy, vyaw] per episode
    def reward(self, data, action, info): ...   # scalar reward — THE cost method
POLICY = Walk
```

`train.py` loads the module-level `POLICY`. `reward()` receives the MJX `data`, the
12-dim `action`, and `info` (`command`, `gait_phase`, `last_action`). Helpers on
`Policy`: `_base_lin_vel`, `_base_ang_vel`, `_projected_gravity`.

## Contract & tuning notes

- `action_scale` (0.5), `gait_frequency` (1.5 Hz), `stand_threshold` (0.05) **must
  match** `config/locomotion.yaml` — the deployed obs/action are built the same way.
- Reward terms in `policies/walk.py` are ported from htwk-gym's K1 ParameterWalk;
  weights are starting points. Getting a robust gait is a tuning exercise (reward
  shaping, domain randomization in `Policy.domain_randomize`, episode length).
- References cloned during design: htwk-gym `envs/K1/parameter_walk.py` (primary),
  booster_gym `play_mujoco.py` / `deploy/deploy.py`.
