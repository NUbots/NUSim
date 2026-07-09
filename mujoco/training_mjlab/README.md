# K1 locomotion training — mjlab

The K1 walk-policy trainer, built on **[mjlab](https://github.com/mujocolab/mjlab)**
(IsaacLab manager API + MuJoCo Warp + rsl_rl/PyTorch).

Provenance (see `PORT_SPEC.md`/`PORT_DECISIONS.md` in the design scratchpad):
- **Walk policy** (`gym` `Base_Walk` 47-obs/12-action contract + `ParameterWalk` reward
  math). This obs/action layout is *bit-for-bit* what the C++ `PolicyBackend::build_obs` already
  implements — so the soccer sim is unchanged (only the ONNX output tensor was renamed `action`→
  `actions` to match rsl_rl).
- **Actuator + articulation = Booster** (`booster_train` `actuator.py`/`booster.py`) — the real K1
  motor table + joint layout, vendored in `assets/k1_constants.py`.

## Setup

```bash
uv sync --extra train      # torch + MuJoCo Warp + rsl_rl (GPU). Heavy; one-time.
```

MuJoCo is standardized on **3.10.0** everywhere (mjlab pins `~=3.10.0`). If you bumped from an older
pin, rebuild the C++ toolchain image once: `./b image` then `./b build`.

## Train / play

```bash
./b train                                  # K1 walk task, GPU, auto-exports ONNX on save
./b train --agent.max-iterations 5000      # tyro overrides pass straight through
./b play --checkpoint-file logs/rsl_rl/k1_walk/<run>/model_<n>.pt
```

`./b train` registers the task, sets `PYTHONPATH`/CUDA, and delegates to `mjlab.scripts.train`.
Checkpoints + an ONNX (`obs[1,47] → actions[1,12]`) land under `logs/rsl_rl/k1_walk/<run>/`.
Drop the `.onnx` in per `config/locomotion.yaml: backend: policy` (build `-DK1_WITH_ONNX=ON`).

## Layout

| Path | Content |
| --- | --- |
| `assets/k1_constants.py` | K1 `EntityCfg`: MJCF via `get_spec()`, v1 `BuiltinPositionActuatorCfg` (gains.yaml kp/kd), ready pose, action scale. Vendored Booster motor table + opt-in `get_k1_hifi_articulation()` (DcMotor + delay). |
| `tasks/k1_walk/env_cfg.py` | 47-obs/12-action walk task: obs group, reward set, commands, terminations, 50 Hz control. |
| `tasks/k1_walk/mdp.py` | Custom terms (gait-clock obs, base_height/torque/power/etc. rewards). |
| `tasks/k1_walk/rl_cfg.py` | rsl_rl PPO (G1 shape: 512·256·128 ELU, adaptive-KL). |
| `tasks/k1_walk/__init__.py` | Registers `Mjlab-Walk-Flat-Booster-K1`. |

## Adding a new task (dribble, kick, …)

`k1_walk` is the template. Every task is a `tasks/k1_<name>/` subpackage that builds a
`ManagerBasedRlEnvCfg` (obs / action / command / reward / termination managers) and registers a
task id. mjlab's manager API means a task is **config, not a new trainer** — you compose terms.

**First decide which kind of task it is — this determines whether the C++ soccer sim changes:**

- **Same deploy contract** (reuses the 47-obs/12-action leg-walk interface — see
  `OBS_ACTION_CONTRACT.md`). You keep the exact obs layout + 12-leg action and only change the
  *reward / command / scene*. The trained ONNX **drops into the existing C++ `PolicyBackend`
  unchanged**. Good for behaviours that are still "walk, but shaped differently" — e.g. a
  dribble where NUbots' behaviour module keeps sending `Move(vx,vy,vyaw)` and the policy just
  walks (ball handling stays high-level). Cheapest path; no C++ change.
- **New contract** (the policy needs to *see* or *do* more). A ball-aware dribble needs **ball
  state in the observation**; a kick needs a **different action regime** (a swing, maybe arms) and
  usually a different episode. That means a bigger obs and/or action vector → you must **also**
  extend the C++ deploy side (`PolicyBackend::build_obs`, a new `kOutputNames`/action mapping,
  possibly a new backend + `config/locomotion.yaml` entry) so the sim can feed/consume the new
  interface, and update `OBS_ACTION_CONTRACT.md`. Train-side and deploy-side move together.

**Steps** (mirror `k1_walk/`):

1. `cp -r tasks/k1_walk tasks/k1_<name>` (drop `__pycache__`).
2. **`env_cfg.py`** — rename the factory (`make_k1_<name>_env_cfg`) and edit the managers:
   - *Observations* — `ObservationGroupCfg(terms={...}, concatenate_terms=True)`. Reuse canned
     terms from `mjlab.envs.mdp` / `mjlab.tasks.velocity.mdp` (`projected_gravity`,
     `joint_pos_rel`, `last_action`, `builtin_sensor`, …); restrict joints with
     `SceneEntityCfg("robot", joint_names=[...])`. **Term dict order = ONNX input order** — keep
     it identical to `build_obs` if you want the same-contract path.
   - *Actions* — `JointPositionActionCfg(actuator_names=..., scale=..., use_default_offset=True)`.
   - *Commands* — `UniformVelocityCommandCfg` for velocity; write a custom `CommandTermCfg` for a
     new command (e.g. target ball position).
   - *Rewards / terminations* — `RewardTermCfg(func=..., weight=..., params={...})`. Reuse mjlab
     terms; put task-specific ones in `mdp.py`.
   - *Scene* — for dribble/kick add the **ball** (and goal) as scene entities/objects in `SceneCfg`
     and add contact/site sensors you reward against (see the `ContactSensorCfg` examples).
3. **`mdp.py`** — a custom term is just a function `func(env, **params) -> torch.Tensor`:
   observation → `[num_envs, dim]`, reward → `[num_envs]`, termination → bool `[num_envs]`. Use
   class-based terms (see `feet_swing`) for stateful ones. Read state off `env.scene["robot"].data`.
4. **`rl_cfg.py`** — usually copy walk's `RslRlOnPolicyRunnerCfg`; rename `experiment_name`.
5. **`__init__.py`** — `register_mjlab_task(task_id="Mjlab-<Name>-Flat-Booster-K1", env_cfg=...,
   play_env_cfg=..., rl_cfg=..., runner_cls=VelocityOnPolicyRunner)`.
6. **`tasks/__init__.py`** — add one line: `from . import k1_<name>`. That's what makes
   `./b train` / `./b play` see it (the launcher imports `training_mjlab.tasks`, registering all).

**Run it:** `./b train Mjlab-<Name>-Flat-Booster-K1` (and `./b play <id> --checkpoint-file …`).
List registered ids: `python -c "import training_mjlab.tasks; from mjlab.tasks.registry import
list_tasks; print(list_tasks())"` (with `PYTHONPATH=mujoco`).

**Deploy:** same-contract task → drop the `.onnx` in as usual. New-contract task → land the paired
`PolicyBackend` change + `OBS_ACTION_CONTRACT.md` update in the same commit, else the sim feeds the
policy the wrong vector with no error (it just misbehaves).

## Contract lock-step (do not drift silently)

The trained ONNX only works in the C++ deploy sim if these match on both sides:
`action_scale = 0.5`, PD gains = `config/gains.yaml`, obs layout = the 47-dim Base_Walk order,
control = 50 Hz. Change one → change its C++ counterpart in the same commit.

## v1 simplifications (fidelity upgrades, not blockers)

- `collision` reward uses whole-trunk self-contact.
- Domain-randomization events (foot friction, encoder bias, base CoM) dropped — need named
  `_collision` geom groups our MJCF lacks. Only reset/push events are wired.
- Actuator = simple PD (gains.yaml). Booster's torque-speed curve + action delay are the opt-in
  `get_k1_hifi_articulation()`, to be enabled *paired with* a matching C++ deploy change.
