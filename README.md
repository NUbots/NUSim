# NUSim

A self-contained **MuJoCo** simulator for the **Booster Robotics K1** humanoid, used by
[NUbots](https://nubook.nubots.net) for RoboCup development. It replaces **both** Booster's gated Webots build
**and** their closed-source `mck` motion runner with one inspectable, NUClear-based simulator that speaks
the **Booster SDK over FastDDS** — the same wire protocol as the real robot — so the
[NUbots](https://github.com/NUbots/NUbots) codebase drives it through its `nusim/*` roles.

```
sim/soccer  (native macOS / Linux docker container, NUClear)
   MuJoCo physics + servo/mode machine + head camera + GLFW viewer + GameController supervisor
        │  Booster SDK over FastDDS (domain 0)            │  camera frames → shared memory
        ▼                                                 ▼
 NUbots  (nusim/* roles: SimK1Walk / SimK1Kick / SimK1GetUp)   input::NUSimCamera → ImageCompressor → NUsight
        ▲
        │  RoboCup GameController (UDP, direct)
```

- **One process** simulates the K1 body — physics, servos, mode machine and fall detection — while the
  locomotion policies run on the NUbots side and drive it through CUSTOM mode + `rt/joint_ctrl`, exactly as
  they drive the real robot. No Booster downloads, no separate motion runner.
- **Vision** is rendered by MuJoCo and handed to NUbots' `input::NUSimCamera` over shared memory, so
  `CompressedImage` reaches **NUsight** exactly as on the real robot.
- **GameController** is heard directly by NUbots over the network; the sim additionally runs a supervisor
  that places the ball/robots per game phase.

## Refereed matches

`./b match` plays a 1v1, 2v2 or 3v3 game with one command: the GameController and an automatic referee run inside the
sim, and every robot gets its own NUbots process with the right team and player number. Linux only.

### Repositories needed

| Repository | Used for |
| --- | --- |
| This one | The sim, GameController, referee and launcher |
| [NUbots](https://github.com/NUbots/NUbots) | The robot code. `./b match` runs its docker image, so it can be anywhere |
| [GameController](https://github.com/RoboCup-HumanoidSoccerLeague/GameController) | Its C API is built into the sim once. Put it next to this checkout, at the commit pinned in [`install_game_controller_api.sh`](mujoco/tools/install_game_controller_api.sh) |

### Running a match

```bash
# Once: build the nusim roles (in NUbots)
./b target generic && ./b configure --set_nusim_roles && ./b build

# Once: build the sim with the GameController API (in this repo, needs cargo)
mujoco/tools/install_game_controller_api.sh   # or pass the path to a GameController checkout
./b configure && ./b build

# Every time
./b match --game 3                            # 1 to 3 a side; starts once every robot connects
./b match --game 1 --away-role <role>         # a different NUbots role for one team
./b match --game 2 --headless                 # no window
```

Robots run `nusim/behaviour` unless told otherwise, and use their own vision and localisation, not ground truth. Their
output is in `mujoco/build-docker/match/robot<id>/log`. Don't run the GameController app at the same time, as both use
port 3939. How it works, settings and limitations are in [MatchReferee](mujoco/module/MatchReferee/README.md).

## Quick start

On **Linux**, requirements are **Docker** (everything else is baked into the image). Full setup, config reference, and
troubleshooting are in **[docs/K1_MUJOCO_SETUP.md](docs/K1_MUJOCO_SETUP.md)**.

```bash
./b configure                   # configure the sim build
./b build                       # build the sim in docker (first run builds the image)
./b run sim/soccer              # launch the soccer sim (viewer + DDS + camera + supervisor)
```

On **macOS**, `./b` builds and runs natively using Apple's OpenGL for the viewer and offscreen camera:

```bash
brew install cmake ninja boost yaml-cpp openjdk@17
export JAVA_HOME="$(brew --prefix openjdk@17)/libexec/openjdk.jdk/Contents/Home"
export PATH="$JAVA_HOME/bin:$PATH"
./mujoco/tools/install_deps.sh   # pinned MuJoCo + DDS libraries + IDL generator
./b configure
./b build
./b test
./b run sim/soccer
```

See [native macOS setup](docs/K1_MUJOCO_SETUP.md#native-macos) for prerequisites, headless rendering and
the networking/camera limitations when connecting to a Linux container. Use `K1SIM_BACKEND=docker`
to select Docker explicitly, or `K1SIM_BACKEND=native` for a native Linux build.

Then drive it from **[NUbots](https://github.com/NUbots/NUbots)** with the `nusim/*` roles:

```bash
cd ~/NUbots
./b target generic
./b configure --set_nusim_roles
./b build
./b run nusim/localisation   # drive with the keyboard while vision and localisation run
# e = walk on/off, w/s/a/d = velocity (tap ~10x), z/x = turn
./b run nusim/behaviour      # autonomous: find the ball by vision and dribble it goalward
./b run nusim/robocup        # the competition role, obeying the GameController
```

The sim ignores the firmware `Move`/`VisualKick` RPCs, so roles built on `skill::Walk` (e.g. the top-level
`keyboardwalk`) won't move the robot. The `nusim/*` roles run walk, kick and get-up as ONNX policies
(`skill::SimK1Walk`, `skill::SimK1Kick`, `skill::SimK1GetUp`) and read the camera with `input::NUSimCamera`.

The Booster SDK **requires** a FastDDS profiles file (participant profile `booster_dds`) or it refuses to
create its DDS participant (`Failed to create participant`). NUbots' `./b run` defaults
`FASTRTPS_DEFAULT_PROFILES_FILE` to its repo's copy, so no flag is needed — see
[Networking](#networking) for the caveat if you pass `--environment` yourself.

## Ground truth and ball commands (testing)

NUSim publishes the ball's and robot's ground truth on `rt/nusim/gt/ball` and `rt/nusim/gt/robot`, and accepts
`rt/nusim/ball_command` to place or roll the ball (all `nav_msgs/Odometry`). NUbots-side tools use them to
validate estimators and to run repeatable shot scenarios. The wire contract is in
[module/SdkBridge/PROTOCOL.md §6](mujoco/module/SdkBridge/PROTOCOL.md).

## Command reference

`./b` selects a native workflow on macOS and the container workflow on Linux; everything after the role name passes straight through to the
binary (`argparse.REMAINDER`), so sim flags are never parsed by `./b`.

| Command | Description |
| --- | --- |
| `./b configure [-i] [--clean] [--set-role R] [--unset-role R]` | CMake-configure with the selected backend; `-i` opens `ccmake`, `--clean` wipes the build dir. |
| `./b build [targets...]` | Ninja build with the selected backend (default: everything; e.g. `./b build sim-soccer`). |
| `./b run <role> [args]` | Run `bin/<role>` natively or in the container. Roles: `sim/soccer`, and `sim/match` once [built](mujoco/module/MatchReferee/README.md). |
| `./b match --game <n> [args]` | Run a refereed match of 1 to 3 a side: the sim plus a hidden NUbots process per robot. See [MatchReferee](mujoco/module/MatchReferee/README.md). |
| `./b roles` | List sim roles and their enabled/disabled state. |
| `./b image` | (Re)build the docker toolchain image. |
| `./b test` | Build and run the C++ unit tests with the selected backend. |

### `sim/soccer` flags

Parsed in [`mujoco/shared/CliOptions.hpp`](mujoco/shared/CliOptions.hpp); `--help` prints the same list.

| Flag | Default | Description |
| --- | --- | --- |
| `--headless` | off | Run without the GLFW viewer window (CI / headless servers). Physics, DDS and the camera bridge all still run. |
| `--field <name>` | `simulation.yaml`'s `field` (`middle`) | Field to play on, one of `simulation.yaml`'s `fields`: `middle` — the RoboCup 2026 Humanoid Soccer League M-Field (14 × 9 m), the field of the Middle Division the K1 plays in; `kidsize` — RoboCup KidSize under the pre-2026 rules (9 × 6 m); `no-field` — a bare flat floor with no field or ball. NUbots' `FieldDescription.yaml` `field_type` must match for localisation. |
| `--game <n>` | none | A match of `n` robots a side, 1–11 (so 2–22 K1s), on `simulation.yaml`'s `game.field` (`middle`). Sets the field and robot count itself, so it can't be combined with `--field` or `--robots`. Team 1 is in the -x half, team 2 is its mirror image; by default each team lines up just off the touchlines facing into the field, two at a time on alternate sides, starting level with its penalty mark and filling towards the halfway line. The main robot is team 1's first. Every robot has independent control, sensors and a DDS domain. `--keyframe` still sets the main robot's pose, moved to its game spot. |
| `--on-field-positions` | off | With `--game`, start in kickoff positions facing the opponent's goal instead of on the touchlines: the attacker (the main robot), then the goalkeeper (2v2+), left wing (3v3+), right wing (4v4+), and the 5th to 11th robots spread evenly through the rest of the team's half (outside the centre circle). Positions are in `simulation.yaml`'s `game`. |
| `--match <file>` | none | With `--game`, load team/player identities and per-robot DDS domains from a validated roster, relative to the config directory unless absolute. See [the multi-robot contract](docs/MULTI_ROBOT_DATA_CONTRACT.md). |
| `--config-dir <dir>` | `mujoco/config` | Config directory to read the YAML from. |
| `--keyframe <name>` | `ready` | Startup keyframe for the **main** robot (e.g. `lying_front` to start fallen and exercise the get-up chain). |
| `--rtf <factor>` | `simulation.yaml`'s `real_time_factor` | Real-time factor; `0` = free-run (uncapped, for tests/sweeps). |
| `--robots <n>` | `1` | **Total** K1s on the field, 1–20. `n−1` extra copies are attached via MuJoCo's `mjSpec` attach API with `subNN_` name prefixes, so the main robot's unprefixed joint/sensor names — and every DDS and shared-memory contract — are untouched. Extras spawn standing on a 5×4 grid clear of the `y = 0` main-robot/ball lane. Each has its own controller and DDS endpoint, holding `ready` until commanded. Sim resets re-place them; `--keyframe` does not apply to them. |
| `--help`, `-h` | — | Print the flag list and exit. |

```bash
./b run sim/soccer --headless                            # no viewer window (CI / server)
./b run sim/soccer --field kidsize                       # the KidSize field instead of the M-Field
./b run sim/soccer --field no-field                      # bare robot on a flat floor, no field/ball
./b run sim/soccer --game 3                              # 3 a side lined up on the M-Field touchlines
./b run sim/soccer --game 5 --on-field-positions         # 5 a side in kickoff positions
./b run sim/soccer --rtf 0                               # free-run (uncapped real-time factor)
./b run sim/soccer --keyframe lying_front                # start fallen, to exercise GetUp
./b run sim/soccer --robots 5                            # 4 extra K1s on the field (max 20 total)
```

### Environment variables

| Variable | Effect |
| --- | --- |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | **Required** by the Booster SDK (see Networking). `./b run` defaults it to the repo's copy. |
| `K1SIM_CONFIG_DIR` | Config directory, same as `--config-dir` (the flag wins). |
| `K1_DDS_UDP_ONLY` | `1` strips the FastDDS shared-memory transport, leaving UDPv4 only — the fallback when the sim and NUbots are on opposite sides of a docker boundary. Equivalent to `udp_only: true` in `config/dds.yaml`. |
| `K1SIM_BACKEND` | `native` or `docker`; defaults to native on macOS and Docker elsewhere. |
| `K1SIM_BUILD_DIR` | Build directory relative to `mujoco/` (default `build-native` or `build-docker` for the selected backend). |

### Viewer keys

Standard MuJoCo mouse camera (left-drag rotate, right-drag pan, scroll zoom, ctrl/shift modifiers for
perturbation drags), plus:

| Key | Action |
| --- | --- |
| `Backspace` | Reset the world and all controllers to startup state, clearing previous commands. |
| `F` | Shove the robot over — a deterministic topple for testing fall detection / `GetUp`, harder than a mouse-drag perturb, which the push-randomised policy usually rides out. |
| `Esc` | Close the window. |

`Space` is **not** a pause: physics stepping belongs to `module::Simulation`'s 1 kHz thread and no pause
switch is exposed yet.

## Networking

The sim speaks the **Booster SDK wire protocol over FastDDS**, so NUbots' `platform::Booster::HardwareIO`
connects to it with no build flag or shim — the same code path it uses against the real robot.

- **DDS domain 0**, because NUbots' `platform::Booster::HardwareIO` hardcodes `ChannelFactory::Init(0)`.
  Configurable in [`mujoco/config/dds.yaml`](mujoco/config/dds.yaml), but changing it means changing NUbots too.
- **Transport** is UDPv4 + shared memory by default. `./b run` gives the container `--network host --ipc host`
  precisely so both survive the docker boundary; set `K1_DDS_UDP_ONLY=1` (or `udp_only: true`) if only UDP works
  in your setup.
- **Topics** — sim publishes `rt/low_state` (IMU + serial/parallel motor state, 50 Hz), `rt/odometer_state`
  and `rt/odom` (the controller's odometry: ground truth, or drifting with the error model in
  [`mujoco/config/odometry.yaml`](mujoco/config/odometry.yaml) enabled), `rt/head_pose`, `rt/fall_down` (on change,
  ≥1 Hz keepalive) and `rt/battery_state` (constant SOC); sim subscribes to
  `rt/joint_ctrl` (`LowCmd`, PD-tracked at 1 kHz in CUSTOM mode). The RPC pair `rt/LocoApiTopicReq` /
  `rt/LocoApiTopicResp` serves `CHANGE_MODE`, `MOVE`, `ROTATE_HEAD`, `LIE_DOWN`, `GET_UP`,
  `GET_UP_WITH_MODE`, `VISUAL_KICK` and `GET_MODE`; unimplemented `api_id`s are accepted with a warning
  (`unknown_api_status` in `dds.yaml`). Full wire details in
  [`mujoco/module/SdkBridge/PROTOCOL.md`](mujoco/module/SdkBridge/PROTOCOL.md).
- **FastDDS profiles are mandatory.** `ChannelFactory::Init(0)` refuses to create its participant unless
  `FASTRTPS_DEFAULT_PROFILES_FILE` points at an XML containing a participant profile named `booster_dds`
  (`Failed to create participant`). `./b run` sets it for you. If you pass `--environment` yourself, note it
  takes ONE comma-separated argument and **replaces** the default — re-include the FastDDS var or all DDS dies.
- **Vision is not DDS.** Head-camera frames are rendered offscreen and written to a Boost.Interprocess
  shared-memory segment (`_boostercamera_head_rgb`, rgb8 544×448 @ 30 Hz) read by NUbots'
  `input::NUSimCamera`. The segment name must match `NUSimCamera.yaml` on the NUbots side. Only the left
  camera is rendered; stereo is future work.

## GameController

`./b match` runs the game inside the sim, with no GameController app: see
[MatchReferee](mujoco/module/MatchReferee/README.md). The rest of this section is about `sim/soccer`.

This is the **RoboCup GameController** (the UDP match-control broadcast), not a USB gamepad — there is no
joypad/SDL input path in NUSim, and manual driving is done from NUbots' `nusim/localisation` role.

- **NUbots hears the GameController itself**, directly and independently over its own socket. That path
  needs nothing from the sim.
- The sim additionally ships a **supervisor** ([`mujoco/module/Supervisor`](mujoco/module/Supervisor)) that
  listens to the same broadcast and moves MuJoCo bodies per game phase — ball to the centre circle on
  kickoff, penalised robots to the sideline — the job Webots' Supervisor role used to do. It parses
  **RoboCup GameControlData protocol version 20** (`RGme` header), receive-only: it never sends a reply.
- **It is OFF by default** (`enabled: false` in [`mujoco/config/supervisor.yaml`](mujoco/config/supervisor.yaml)).
  With `--network host`, NUbots binds GameController port **3838** on the same host; a second binder steals
  it and breaks NUbots with `Unable to bind the UDP socket: Address already in use`. Enable the supervisor
  only when running the sim **standalone**, with no NUbots GameController on the host.
- With no GameController on the network it idles silently — no errors either way.
- Ball placement, per-robot team/player mapping, home and penalty poses, and which events reset the ball
  (`finished`, `goal`, `half_change`) are all configurable in `supervisor.yaml`. Exact RoboCup per-player
  penalty spacing is **not** implemented — the penalty spot is a single configurable pose.

## Locomotion policy

NUSim does **one thing: simulate**. It neither trains nor runs locomotion policies: the sim is a
servo-command listener (CUSTOM mode + `rt/joint_ctrl` LowCmd, PD-tracked at 1 kHz), and inference runs on
the NUbots side (`module/skill/SimK1Walk`, `SimK1Kick` and `SimK1GetUp`). Policies are trained in the NUbots
**[mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground)** (branch `feat/k1-training`,
`K1JoystickFlatTerrain` / `K1Getup` tasks) and exported to ONNX with `learning/export_k1_onnx.py`. The
walk observation/action interface is pinned in
**[docs/OBS_ACTION_CONTRACT.md](docs/OBS_ACTION_CONTRACT.md)** — anything that trains a walk policy for
the K1 must match it.

See [the MJWarp assessment](docs/MJWARP_ASSESSMENT.md) for the local CPU comparison and why native
MuJoCo remains the default for this interactive simulator.

## Layout

| Path | Description |
| --- | --- |
| `b`, `mujoco/b.py`, `mujoco/tools/` | NUbots-style `./b` command dispatcher (`run`, `build`, `configure`, `roles`, `image`). |
| `mujoco/roles/` | Role files (`sim/soccer.role`) → `bin/<role>` binaries. |
| `mujoco/module/` | NUClear modules: `Simulation`, `SdkBridge` (DDS), `Locomotion`, `Camera`, `Supervisor`, `Viewer`. |
| `mujoco/models/k1/` | Vendored MuJoCo K1 model (BSD-3, `booster_assets`) + RoboCup/flat scenes. |
| `mujoco/docker/` | Toolchain image + `k1sim.sh` (the container workflow `./b` wraps). |
| `docs/K1_MUJOCO_SETUP.md` | Setup, config reference, end-to-end with NUbots, and troubleshooting. |
| `docs/OBS_ACTION_CONTRACT.md` | ONNX policy interface contract (obs/action layout the sim expects). |
| [docs/MULTI_ROBOT_DATA_CONTRACT.md](docs/MULTI_ROBOT_DATA_CONTRACT.md) | Multi-robot roster, independent control and sensors, and planned ground-truth snapshots. |
| `mujoco/module/SdkBridge/PROTOCOL.md` | Booster SDK DDS wire surface: topics, message layouts, RPC `api_id`s. |

Forked from [NUWebots](https://github.com/NUbots/NUWebots); the Webots/NUgus simulation has been removed in
favour of the MuJoCo path (see git history if you need it).

### Native motion-policy players

See [Native motion-policy players](docs/NATIVE_MOTION_PLAYER.md) for the companion CPU
player build and behavioural checks using independent robot domains and simulated localisation.
