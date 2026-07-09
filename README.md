# NUSim

A self-contained, docker-based **MuJoCo** simulator for the **Booster Robotics K1** humanoid, used by
[NUbots](https://nubots.net) for RoboCup development. It replaces **both** Booster's gated Webots build
**and** their closed-source `mck` motion runner with one inspectable, NUClear-based simulator that speaks
the **Booster SDK over FastDDS** — the same wire protocol as the real robot — so `NUbots_K1` binaries drive
it **unchanged**.

```
sim/soccer  (docker container, NUClear)
   MuJoCo physics + RL locomotion policy + head camera + GLFW viewer + GameController supervisor
        │  Booster SDK over FastDDS (domain 0)            │  camera frames → shared memory
        ▼                                                 ▼
 NUbots_K1  (walk / behaviour / robocup roles)     input::K1Camera → ImageCompressor → NUsight
        ▲
        │  RoboCup GameController (UDP, direct)
```

- **One process** simulates the K1 body *and* owns its locomotion (a trained velocity-tracking RL policy),
  so NUbots' high-level `Move`/`RotateHead`/`GetUp` commands produce real dynamic motion — no Booster
  downloads, no separate runner.
- **Vision** is rendered by MuJoCo and handed to NUbots' *unchanged* `input::K1Camera` over shared memory,
  so `CompressedImage` reaches **NUsight** exactly as on the real robot.
- **GameController** is heard directly by NUbots over the network; the sim additionally runs a supervisor
  that places the ball/robots per game phase.

## Quick start

Requirements: **Docker** (everything else is baked into the image). Full setup, config reference, and
troubleshooting are in **[docs/K1_MUJOCO_SETUP.md](docs/K1_MUJOCO_SETUP.md)**.

```bash
./b configure && ./b build      # build the sim in docker (first run builds the image)
./b run sim/soccer              # launch the soccer sim (viewer + DDS + camera + supervisor)
```

Then drive it from **[`NUbots_K1`](https://github.com/NUbots/NUbots_K1)** exactly as against the real robot
(e.g. `./b run keyboardwalk`). See the setup doc for the required FastDDS profile env var.

## Training a locomotion policy

The default locomotion backend is a non-dynamic placeholder; real walking comes from a trained policy.
Training is **[mjlab](https://github.com/mujocolab/mjlab)** (MuJoCo Warp + rsl_rl), GPU-parallel in the
same MuJoCo physics used for deployment (no sim2sim gap):

```bash
uv sync --extra train           # torch + MuJoCo Warp + rsl_rl (one-time)
./b train                       # train the K1 walk policy (auto-exports ONNX on save)
./b play --checkpoint-file <ckpt.pt>   # roll out a trained policy in the viewer
```

The trained policy exports to ONNX (`obs[1,47] → actions[1,12]`), loaded by the C++ locomotion backend
(`config/locomotion.yaml: backend: policy`, built `-DK1_WITH_ONNX=ON`). Setup, task, and the obs/action
contract are under **[mujoco/training_mjlab/](mujoco/training_mjlab/)**. Defining your own task beyond walk
(dribble, kick, …) is documented there under **Adding a new task**.

## Layout

| Path | Description |
| --- | --- |
| `b`, `mujoco/b.py`, `mujoco/tools/` | NUbots-style `./b` command dispatcher (`run`, `build`, `configure`, `image`, `train`, `play`). |
| `mujoco/roles/` | Role files (`sim/soccer.role`, `sim/training.role`) → `bin/<role>` binaries. |
| `mujoco/module/` | NUClear modules: `Simulation`, `SdkBridge` (DDS), `Locomotion`, `Camera`, `Supervisor`, `Viewer`. |
| `mujoco/models/k1/` | Vendored MuJoCo K1 model (BSD-3, `booster_assets`) + RoboCup/flat scenes. |
| `mujoco/training_mjlab/` | Training: mjlab (MuJoCo Warp + rsl_rl), K1 walk task, contract + Booster actuators, ONNX export. |
| `mujoco/docker/` | Toolchain image + `k1sim.sh` (the container workflow `./b` wraps). |
| `docs/K1_MUJOCO_SETUP.md` | Setup, config reference, end-to-end with `NUbots_K1`, and troubleshooting. |

Forked from [NUWebots](https://github.com/NUbots/NUWebots); the Webots/NUgus simulation has been removed in
favour of the MuJoCo path (see git history if you need it).
