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

Then drive it from **[`NUbots_K1`](https://github.com/NUbots/NUbots_K1)** exactly as against the real robot.
The Booster SDK **requires** a FastDDS profiles file (participant profile `booster_dds`) or it refuses to
create its DDS participant (`Failed to create participant`):

```bash
cd ~/NUbots_K1
./b run keyboardwalk --environment FASTRTPS_DEFAULT_PROFILES_FILE=/home/nubots/NUbots/tools/fastdds_default_profiles.xml
# e = walk on/off, w/s/a/d = velocity (0.01 m/s per press — tap ~10x), z/x = turn, arrows = head
```

See [docs/K1_MUJOCO_SETUP.md](docs/K1_MUJOCO_SETUP.md) for the full end-to-end walkthrough.

## Locomotion policy

NUSim does **one thing: simulate**. It does not train policies. Real walking comes from a trained
velocity-tracking RL policy in ONNX form, produced by the NUbots
**[mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground)** (branch `feat/k1-training`,
`K1JoystickFlatTerrain` task). Drop the exported `.onnx` in, set `config/locomotion.yaml: backend: policy`
+ `policy.path`, and build with `-DK1_WITH_ONNX=ON`. The exact observation/action interface the sim
expects is pinned in **[docs/OBS_ACTION_CONTRACT.md](docs/OBS_ACTION_CONTRACT.md)** — anything that
trains a policy for this sim must match it.

## Layout

| Path | Description |
| --- | --- |
| `b`, `mujoco/b.py`, `mujoco/tools/` | NUbots-style `./b` command dispatcher (`run`, `build`, `configure`, `image`). |
| `mujoco/roles/` | Role files (`sim/soccer.role`) → `bin/<role>` binaries. |
| `mujoco/module/` | NUClear modules: `Simulation`, `SdkBridge` (DDS), `Locomotion`, `Camera`, `Supervisor`, `Viewer`. |
| `mujoco/models/k1/` | Vendored MuJoCo K1 model (BSD-3, `booster_assets`) + RoboCup/flat scenes. |
| `mujoco/docker/` | Toolchain image + `k1sim.sh` (the container workflow `./b` wraps). |
| `docs/K1_MUJOCO_SETUP.md` | Setup, config reference, end-to-end with `NUbots_K1`, and troubleshooting. |
| `docs/OBS_ACTION_CONTRACT.md` | ONNX policy interface contract (obs/action layout the sim expects). |

Forked from [NUWebots](https://github.com/NUbots/NUWebots); the Webots/NUgus simulation has been removed in
favour of the MuJoCo path (see git history if you need it).
