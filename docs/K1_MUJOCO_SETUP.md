# Booster K1 — MuJoCo Simulation Setup

How to run the **Booster Robotics K1** in the self-contained MuJoCo simulator (`mujoco/`) and drive it
from the NUbots codebase — no Booster downloads, no gated Webots build, no separate `mck` process.

---

## How it works (read this first)

The MuJoCo sim replaces **both** pieces to the left of the DDS boundary in the old Webots setup — Booster's
Webots build *and* the `mck` motion runner — with one NUClear-based binary that speaks the **same** Booster
SDK wire protocol (FastDDS, domain 0):

```
sim/soccer  (docker container, NUClear)
   physics (MuJoCo, 1 kHz) + locomotion policy + head camera + GameController supervisor + GLFW viewer
        │  Booster SDK over FastDDS (domain 0)          │  camera frames → shared memory
        ▼                                               ▼
 NUbots_K1  platform::Booster::HardwareIO         input::K1Camera → ImageCompressor → NUsight
        ▲   (B1LocoClient: Move / RotateHead / …)
        │  RoboCup GameController (UDP 3838, direct)
```

- **`sim/soccer`** (the role binary) owns the physics (`module::Simulation`), the mode state machine +
  locomotion backends (`module::Locomotion`), the DDS publishers/RPC server (`module::SdkBridge`), the head
  camera → shared-memory bridge (`module::Camera`), the GameController-aware body-placement supervisor
  (`module::Supervisor`), and the GLFW viewer (`module::Viewer`) — one process, all in this repo.
- **Locomotion**: NUbots sends only high-level `Move(vx,vy,vyaw)`; the sim owns the gait. The default
  backend is a non-dynamic kinematic glide; real dynamic walking comes from a trained RL policy
  (`backend: policy`) — see [Getting a locomotion policy](#7-getting-a-locomotion-policy). Both consume the
  same `Move` command.
- **NUbots** (the [`NUbots_K1`](https://github.com/NUbots/NUbots_K1) repo) connects over DDS exactly as it
  does today: same topics, same RPC surface, same `platform::Booster::HardwareIO` role wiring documented in
  [docs/K1_WEBOTS_SETUP.md](K1_WEBOTS_SETUP.md). **No changes to NUbots_K1 roles are required** to switch
  from the Webots+`mck` sim to this one.
- Unlike the Webots setup, there is **no Booster blob anywhere in the loop** — no gated wiki downloads, no
  closed-source runner. The Booster SDK is used only as a *wire-protocol reference* (see
  `mujoco/module/SdkBridge/PROTOCOL.md`) and, separately, as a *test client* in the contract tests.

## 1. Requirements

| | |
| --- | --- |
| OS | Linux with **Docker** (the toolchain and all dependencies are baked into a docker image — nothing to install natively) |
| GPU | Optional. `nvidia-container-toolkit` gives GPU-accelerated rendering (`--gpus all`); otherwise the container falls back to `/dev/dri` (Mesa). The sim also runs fully **headless** on a machine with no GPU/display at all. |
| Display | Optional. Only needed for the GLFW viewer window; `--headless` (or `K1_HEADLESS=1`, see below) skips it entirely — useful for CI or a bare server. |

Nothing else to install for the **sim** itself: `mujoco/docker/k1sim.sh` builds the image (Ubuntu 22.04 +
pinned MuJoCo/Fast-DDS/NUClear/GLFW versions — see `mujoco/tools/install_deps.sh`) the first time it's
needed. The host-side `./b` formatters use [uv](https://docs.astral.sh/uv/) — see
[Host tooling & dependencies](#host-tooling--dependencies-uv) below.

## 2. Quick start

```bash
# from this repo — NUbots-style ./b workflow
./b configure && ./b build      # build the sim in docker (first run builds the image)
./b run sim/soccer              # launch the soccer sim (viewer + DDS + camera + supervisor)
```

`./b run <role>` execs `bin/<role>` in the container with X11 + GPU + `--network host --ipc host` (DDS)
passthrough. Roles: `sim/soccer` (full sim). Args after the role pass through to the binary:

```bash
./b run sim/soccer --headless                            # no viewer window (CI / server)
./b run sim/soccer --model models/k1/k1_scene_flat.xml   # bare robot on a flat floor, no field/ball
./b run sim/soccer --rtf 0                                # free-run (uncapped real-time factor)
```

> `--model` must point at a **scene** (`k1_scene_robocup.xml`, `k1_scene_flat.xml`), not the bare
> `K1_22dof.xml` component (no floor/lights → black screen).

> `K1_MODEL` must point at a **scene** (`k1_scene_robocup.xml`, `k1_scene_flat.xml`, or your own).
> `K1_22dof.xml` is the robot *component* for `<include>`: standalone it has no floor and no lights,
> so the robot free-falls out of view and the viewer renders black.

Then, exactly as with the Webots setup, drive it from **`NUbots_K1`**:

> **Required:** the Booster SDK's `ChannelFactory::Init(0)` (what `platform::Booster::HardwareIO`
> calls) refuses to create its DDS participant unless `FASTRTPS_DEFAULT_PROFILES_FILE` points at a
> profiles XML containing a participant profile named **`booster_dds`**. `NUbots_K1` ships one at
> `tools/fastdds_default_profiles.xml`; pass it with `--environment` as shown below (path as seen
> *inside* the container, where the repo mounts at `/home/nubots/NUbots`).

```bash
cd ~/NUbots_K1
./b target generic
./b configure
./b build -- bin/keyboardwalk   # the TOP-LEVEL keyboardwalk role (see note below)
./b run keyboardwalk --environment FASTRTPS_DEFAULT_PROFILES_FILE=/home/nubots/NUbots/tools/fastdds_default_profiles.xml
# focus this terminal: e = walk on/off, w/s/a/d = velocity, z/x = turn, arrows = head
```

Use the **top-level `keyboardwalk` role**, not `webots/keyboardwalk`: upstream's `roles/webots/*.role`
still load the legacy NUgus TCP modules (`platform::Webots`), while the top-level roles use
`platform::Booster::HardwareIO` + the K1 skills — the same binary that runs on the real robot.
(Building by role *name* can resolve to the wrong same-named target, e.g. `fake/keyboardwalk` —
build the explicit output `bin/keyboardwalk`.)

`./b run` launches the container with `--network host` **and `--ipc=host`** (see patch 9 in
[K1_WEBOTS_SETUP.md](K1_WEBOTS_SETUP.md#building-nubots_k1-generic-from-scratch--required-patches)), so its
DDS reaches `k1_mujoco_sim` on the host (domain 0) the same way it reached `mck` before. No further NUbots_K1
patches beyond what that doc already describes are needed — the wire protocol is unchanged.

### Autonomous behaviour / dribble test

The full vision→localisation→behaviour stack runs against the sim too (verified: the robot finds the ball
by vision and dribbles it goalward autonomously):

```bash
# terminal 1 — the sim (confirm "Camera: rendering 640 x 480" appears)
cd ~/NUSim && ./b run sim/soccer
# terminal 2 — the Tester purpose (find_ball / walk_to_ball / align_ball_to_goal)
cd ~/NUbots_K1
./b build -- bin/test/behaviour
./b run test/behaviour --environment=FASTRTPS_DEFAULT_PROFILES_FILE=/home/nubots/NUbots/tools/fastdds_default_profiles.xml
```

Two NUbots_K1-side requirements, both easy to miss:

- The role **must use `skill::K1Walk`, not `skill::Walk`** — upstream `Walk` is the NUgus joint-trajectory
  engine whose servo output the Booster runner ignores; only `K1Walk` translates `Walk` tasks into the
  Booster `Move` RPC. (`roles/test/behaviour.role` and `roles/keyboardwalk.role` are already correct.)
- `VisualMesh.yaml` needs a `cameras:` entry whose key matches the camera **name** in `K1Camera.yaml`
  ("Left Camera") — with no entry the mesh silently drops every frame
  (`VisualMesh Stats: ... Processing 0/s`).

## Host tooling & dependencies (uv)

There are **two separate environments** — keep them straight:

| Environment | Manages | Lives in | Used by |
| --- | --- | --- | --- |
| **docker image** | C++ sim toolchain: cmake/ninja, the **MuJoCo C library**, Fast-DDS, NUClear (`tools/install_deps.sh`) | the `k1sim` image | `./b configure` / `build` / `run` |
| **host uv venv** | Python: the `./b` formatters | repo-root `.venv` (`pyproject.toml` + `uv.lock`) | formatters |

The Python dependency manager is [uv](https://docs.astral.sh/uv/) (as in NUbots). From the repo root:

```bash
uv sync                 # host tooling (formatters) — light
```

The C++ deploy MuJoCo version is pinned in `cmake/MuJoCoTarget.cmake`, `docker/Dockerfile`, and
`tools/install_deps.sh`. Keep the training side (the mujoco_playground fork, §7) on the same MuJoCo
version to avoid a sim2sim gap; bumping one means bumping **all** and rebuilding the image (`./b image`).

### Extra `./b` commands

- **`./b image`** — (re)build the docker toolchain image. `./b build` only builds it when it's *missing*, so
  after changing a baked dependency (e.g. the MuJoCo version) you must run this explicitly.
- **`./b configure --clean`** — wipe the build dir (`build-docker/`, incl. `CMakeCache.txt`) before
  configuring. Needed when a cached path goes stale — e.g. cmake caches `MUJOCO_INCLUDE_DIR-NOTFOUND` after a
  version bump and keeps failing until it's cleared.

## 3. The mode / locomotion story

The sim owns a Booster-style mode state machine (`module::Locomotion`): `ChangeMode`/`Move`/`RotateHead`/
`GetUp`/kick RPCs drive it exactly like the real robot's firmware. Two interchangeable **backends** compute
joint targets once in `WALKING`/`SOCCER` mode (`config/locomotion.yaml`, `backend:`):

- **`policy` (the default; a trained policy ships).** Feeds a 50 Hz ONNX policy (trained in the
  mujoco_playground fork — see §7) that outputs PD target offsets, driven purely through the same 22
  torque actuators as the real robot: dynamics-faithful walking. The shipped policy lives at
  `models/k1/policies/k1_walk.onnx` (`config/locomotion.yaml`'s `policy.path`; requires the ONNX build,
  `K1SIM_CMAKE_ARGS="-DK1_WITH_ONNX=ON"`). The interface the sim expects is pinned in
  [OBS_ACTION_CONTRACT.md](OBS_ACTION_CONTRACT.md) — retraining/replacing the policy must match it.
- **`kinematic` (fallback).** Directly servos the robot's root velocity/height/upright attitude from the
  commanded `vx/vy/vyaw`, with a PD-held ready pose and a procedural stepping animation on top. **Not
  dynamics-faithful** — the robot doesn't fall over from being pushed off-balance by its own gait, because
  the root is velocity-servoed rather than driven purely by joint torques and contact forces. Useful when
  the ONNX build is unavailable or a deterministic glide is preferable.

The sim boots holding **PREPARE** (standing at the ready pose) until the first `ChangeMode` arrives
(`locomotion.yaml`'s `initial_mode`; set `damping` for the real robot's limp-at-boot behaviour — but note
the RL policy cannot get up from a collapsed start).

Getting knocked over is still meaningful either way: `module::SdkBridge` reports `rt/fall_down` from the
base's actual tilt (`config/locomotion.yaml`'s `fall:` thresholds), and `GetUp`/`GetUpWithMode` run a
scripted recovery. In the viewer, **double-click a body then Ctrl+right-drag** to apply a push force and
test this interactively (see `module/Viewer` below). The sim also logs a base-pose heartbeat
(`Simulation: t = ..., base x = ...`) every 5 s of sim time, so headless runs show whether the robot is
actually moving.

## 4. Config files reference (`mujoco/config/`)

| File | Owns |
| --- | --- |
| `simulation.yaml` | `module::Simulation` — model path, real-time factor, state-publish rate |
| `gains.yaml` | Per-joint PD stiffness/damping + ready pose (seeded from Booster's official K1 deploy config) |
| `locomotion.yaml` | `module::Locomotion` — mode/backend selection, kinematic servo gains, getup/liedown/kick timings, fall thresholds |
| `dds.yaml` | `module::SdkBridge` — DDS domain, UDP-only fallback, battery SOC, unknown-RPC status |

All are read at startup (`--config-dir` or `$K1SIM_CONFIG_DIR` to point elsewhere); `--model`/`--rtf` on the
command line override the corresponding YAML value for one-off runs (this is what `K1_MODEL`/`K1_RTF` above
set).

## 5. The viewer (`module::Viewer`)

A GLFW + MuJoCo GPU-rendered window, skipped entirely under `--headless`. Standard `simulate`-style controls:

- **Left-drag**: rotate camera. **Right-drag**: pan. **Scroll**: zoom.
- **Double-click** a body to select it, then **Ctrl+right-drag** to push it (translate) or **Ctrl+left-drag**
  to twist it (rotate) — handy for shoving the robot over to test `fall_down`/`GetUp` recovery.
- **Esc** closes the window (shuts the whole sim down). **Overlay** (top-left) shows sim time, measured
  real-time factor, and the current mode.
- Pausing physics from the viewer is **not** wired up (that's `module::Simulation`'s pacing thread, not the
  viewer's, and there's currently no pause switch to hook into) — noted here as future work, not a bug.

## 6. Camera → NUsight (`module::Camera`)

`sim/soccer` renders the K1's head camera (a `<camera name="head">` in the model) offscreen and writes rgb8
frames into a **Boost.Interprocess shared-memory segment** (`_boostercamera_head_raw_rgb` — the left-camera
entry in NUbots_K1's `K1Camera.yaml`) laid out exactly like NUbots' `input::K1Camera` `SharedImageHeader`.
So the sim impersonates NUbridge: the **unchanged** NUbots `robocup`/`behaviour` role reads the segment →
`ImageCompressor` → `NetworkForwarder` → **NUsight** shows `CompressedImage`, same as on the real robot.
`--ipc host` (already used by `./b run` and NUbots' `./b run`) shares `/dev/shm` across the containers.
Config: `mujoco/config/camera.yaml` (segment name, resolution, fps, intrinsics). Renders via **EGL**
(offscreen, no window), so it works **headless** too — just needs a render device (`./b run` passes
`/dev/dri` + GPU). No device ⇒ logs and disables, no crash — but then vision receives **zero** frames
(`VisualMesh Stats: Receiving 0/s`): confirm the sim log shows `Camera: rendering 640 x 480 ...` before
blaming the NUbots side. The right-camera segment (`_boostercamera_head_raw_right_rgb`) is not rendered
yet; K1Camera warn-retries on it harmlessly (stereo is future work).

## 7. Getting a locomotion policy

NUSim does not train policies — it only simulates. A trained walk policy **ships** at
`mujoco/models/k1/policies/k1_walk.onnx` (200M-step PPO, `K1JoystickFlatTerrain`, domain randomization).
To retrain or replace it: policies are trained in the NUbots
**[mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground)** (branch `feat/k1-training`,
MJX/brax PPO, `K1JoystickFlatTerrain` / `K1JoystickRoughTerrain` tasks) —
`learning/train_jax_ppo.py --env_name=K1JoystickFlatTerrain --domain_randomization`, then export a
checkpoint with `learning/export_k1_onnx.py` (bakes the brax observation normalization into the graph).
The C++ `PolicyBackend` loads the ONNX (`config/locomotion.yaml: backend: policy`, build
`-DK1_WITH_ONNX=ON`). Anything that trains a policy for this sim must match the interface pinned in
**[OBS_ACTION_CONTRACT.md](OBS_ACTION_CONTRACT.md)**; a quick sim2sim sanity check for an exported policy
is `test/contract/policy_walk_check.py --onnx <file> --vx 0.3` (pure-python PolicyBackend replica,
reports displacement + uprightness).

## 8. GameController supervisor (`module::Supervisor`)

`sim/soccer` watches the RoboCup GameController UDP broadcast (port 3838) and places physics bodies per game
phase (ball to centre on kickoff, penalised robots to the sideline) — the sim-side supervisor Webots
provided. NUbots hears the GameController **directly and independently**; the sim never replies to it. No GC
on the network ⇒ idle no-op. Config: `mujoco/config/supervisor.yaml`.

## Troubleshooting

- **`webots/keyboardwalk` connects but the K1 doesn't move / no `LowState`.** Same FastDDS shared-memory
  caveat as the old Webots+`mck` setup: make sure `./b run` used `--ipc=host` (see
  [K1_WEBOTS_SETUP.md](K1_WEBOTS_SETUP.md), patch 9). If it still doesn't come through, force UDP-only
  transport on the sim side with `K1_DDS_UDP_ONLY=1` (or `config/dds.yaml`'s `udp_only: true`).
- **`Failed to open segment fast_datasharing_... -> Function open_and_init_shared_segment_notification`,
  then `request timed out`.** The client container can't reach the sim's `/dev/shm` segments. Two causes:
  (a) the NUbots side wasn't launched with `--ipc host` (patch 9, see above); (b) stale segments from a
  crashed run — stop both sides and `rm -f /dev/shm/fast_datasharing* /dev/shm/fastrtps_*`.
- **Keyboardwalk UI works but the robot never leaves PREPARE (no mode change).** Transport mismatch:
  don't run the sim UDP-only (`K1_DDS_UDP_ONLY=1`) while the NUbots container has `--ipc host` (or vice
  versa) — asymmetric transports let discovery succeed but silently blackhole the RPCs. Use the same
  transport on both sides: both SHM (default + `--ipc host`) or both UDP-only.
- **No window / `xhost`/X11 authorization errors.** `./b run` runs `xhost +local:` for you when `$DISPLAY`
  is set; on a remote/SSH session without X forwarding, run headless (`./b run sim/soccer --headless`) or
  forward X11 (`ssh -X`).
- **No GPU / rendering looks software-y.** The container falls back to Mesa software or integrated-GPU
  rendering via `/dev/dri` when `nvidia-container-toolkit` isn't installed — slower, but functional; the
  physics and DDS surface are unaffected either way. Headless mode sidesteps rendering entirely.
- **`MuJoCo <version> not found` during `./b configure`.** The docker image still has the old MuJoCo C
  library baked in. Rebuild it: `./b image`, then `./b configure --clean && ./b build` (`--clean` clears
  cmake's cached `NOTFOUND`). Pointing `-DMUJOCO_DIR=...` only helps if a matching install already exists
  inside the container.
- **`docker/k1sim.sh build` fails on a fresh machine.** First run builds the toolchain image
  (`mujoco/docker/Dockerfile`), which can take a few minutes; check `docker images | grep k1sim` if it seems
  stuck, and re-run — layers are cached after the first build.

## Known limitations

- **Camera needs a render device.** `module::Camera` renders offscreen via EGL — works headless, but needs
  `/dev/dri` or an NVIDIA device (both passed by `./b run`). With no device it disables gracefully (and
  vision on the NUbots side starves — see §6).
- **Mono camera only.** The K1 has stereo head cameras; the sim renders the left one. NUbots' K1Camera
  retries the right segment forever (harmless warning spam).
- **Single robot, one DDS domain.** One robot on domain 0. A multi-robot field needs one sim process per
  robot, each on its own domain.
- **Render fidelity vs vision networks.** The field is a procedural green checker with box-geom lines and a
  plain orange ball (the Webots textures aren't redistributable). The Webots-trained visual-mesh network and
  the RoboCup-imagery YOLO work on these renders, but detection margins are thinner than on real imagery —
  detector thresholds on the NUbots side may need loosening.
