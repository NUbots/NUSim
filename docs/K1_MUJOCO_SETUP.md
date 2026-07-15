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

- **`sim/soccer`** (the role binary) owns the physics (`module::Simulation`), the reduced mode machine +
  LowCmd servo tracking (`module::Locomotion`), the DDS publishers/RPC server (`module::SdkBridge`), the head
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

# One-time: the image's OpenVINO has no CPU device (built ENABLE_INTEL_CPU=OFF); the
# policy skills need one. Mount the official 2024.6.0 runtime over it:
#   curl -L -o /tmp/openvino.tgz https://storage.openvinotoolkit.org/repositories/openvino/packages/2024.6/linux/l_openvino_toolkit_ubuntu22_2024.6.0.17404.4c0f47d2335_x86_64.tgz
#   mkdir -p /tmp/ov_overlay && tar xzf /tmp/openvino.tgz -C /tmp/ov_overlay --strip-components=1

./b run --volume /tmp/ov_overlay/runtime/lib/intel64:/usr/local/runtime/lib/intel64:ro \
    keyboardwalk \
    --environment "FASTRTPS_DEFAULT_PROFILES_FILE=/home/nubots/NUbots/tools/fastdds_default_profiles.xml,LD_LIBRARY_PATH=/usr/local/runtime/lib/intel64"
# focus this terminal: e = walk on/off, w/s/a/d = velocity, z/x = turn, arrows = head
```

> **`--environment` takes ONE comma-separated argument.** Passing a second
> `--environment` flag silently replaces the first — and losing
> `FASTRTPS_DEFAULT_PROFILES_FILE` kills the whole Booster SDK participant (`Failed to
> create participant`): vision keeps running off shared memory while LowState/RPCs
> silently vanish, and the robot collapses when a policy skill switches to CUSTOM with
> nothing streaming. (The sim now PD-holds the entry pose in that case, but the robot
> still won't move.) Check the K1 log for `Loaded walk policy` + no
> `Failed to get current mode` spam before debugging anything else.

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
./b run --volume /tmp/ov_overlay/runtime/lib/intel64:/usr/local/runtime/lib/intel64:ro \
    test/behaviour \
    --environment "FASTRTPS_DEFAULT_PROFILES_FILE=/home/nubots/NUbots/tools/fastdds_default_profiles.xml,LD_LIBRARY_PATH=/usr/local/runtime/lib/intel64"
```

Two NUbots_K1-side requirements, both easy to miss:

- The role **must use `skill::K1WalkPolicy` (+ `skill::K1GetUpPolicy`), not `skill::Walk`** — upstream
  `Walk` is the NUgus joint-trajectory engine whose servo output goes nowhere on the K1; the policy skills
  run the ONNX locomotion policies and stream `rt/joint_ctrl`.
  (`roles/test/behaviour.role` and `roles/keyboardwalk.role` are already correct.)
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

**Locomotion policies live in NUbots_K1, not in the sim.** The sim is a servo-command listener: the NUbots
stack runs ONNX inference on its side (`module/skill/K1WalkPolicy` for walking, `module/skill/K1GetUpPolicy`
for fall recovery — OpenVINO, 50 Hz) and streams the resulting joint targets over the Booster SDK's
low-level topic (`rt/joint_ctrl`, `LowCmd`), which the sim PD-tracks at 1 kHz in **CUSTOM** mode with the
per-motor `kp`/`kd`/`tau` carried in each message (torques clamped to the model's motor `forcerange`).

`module::Locomotion` keeps only a reduced Booster-style mode machine:

- **DAMPING** — motors limp (zero ctrl).
- **PREPARE** — cubic blend to and PD-hold of the `gains.yaml` ready pose (boot convenience: the sim spawns
  standing, and idling limp would just collapse before a client connects). `RotateHead` still steers the
  head here.
- **CUSTOM** — PD-track the latest `rt/joint_ctrl` LowCmd (the path every NUbots_K1 policy uses).

For SDK wire compatibility the old high-level RPCs are still answered: `ChangeMode(WALKING/SOCCER)` maps to
PREPARE with a warning, and `Move`/`GetUp`/`LieDown`/`VisualKick` are accepted but ignored (warn-once) —
drive the robot with CUSTOM + LowCmd instead.

The sim boots holding **PREPARE** until the first `ChangeMode` arrives (`locomotion.yaml`'s `initial_mode`;
set `damping` for the real robot's limp-at-boot behaviour). `--keyframe lying_front` spawns the robot on
the floor instead — the standard way to exercise the NUbots get-up policy end to end.

Getting knocked over is still meaningful: `module::SdkBridge` reports `rt/fall_down` from the base's actual
tilt/height (`config/locomotion.yaml`'s `fall:` thresholds), which is what triggers the NUbots
FallRecovery → GetUpPlanner → `K1GetUpPolicy` chain. In the viewer, **double-click a body then
Ctrl+right-drag** to apply a push force and test this interactively (see `module/Viewer` below). The sim
also logs a base-pose heartbeat (`Simulation: t = ..., base x = ...`) every 5 s of sim time, so headless
runs show whether the robot is actually moving.

## 4. Config files reference (`mujoco/config/`)

| File | Owns |
| --- | --- |
| `simulation.yaml` | `module::Simulation` — model path, real-time factor, state-publish rate |
| `gains.yaml` | Per-joint PD stiffness/damping + ready pose (seeded from Booster's official K1 deploy config) |
| `locomotion.yaml` | `module::Locomotion` — initial mode, prepare blend time, fall thresholds |
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
- **F** shoves the robot over (adds root velocity under the sim mutex) — deterministic fall for testing
  FallRecovery/GetUp; mouse-drag perturbs are usually within what the push-randomised policy survives.
- **Backspace** resets the simulation to its startup state (the model's `ready` keyframe: robot pose, ball
  position, all velocities). Physics state only — the Locomotion controller keeps its current mode and last
  commands, like picking a real robot up and placing it back on the start mark mid-program. (Viewer emits
  `SimResetRequest`; `module::Simulation` handles it, so headless/scripted resets can emit the same message.)
- Pausing physics from the viewer is **not** wired up (that's `module::Simulation`'s pacing thread, not the
  viewer's, and there's currently no pause switch to hook into) — noted here as future work, not a bug.

## 6. Camera → NUsight (`module::Camera`)

`sim/soccer` renders the K1's head camera (a `<camera name="head">` in the model) offscreen and writes rgb8
frames into a **Boost.Interprocess shared-memory segment** (`_boostercamera_head_rgb` — the left-camera
entry in NUbots_K1's `K1Camera.yaml`; NUbridge dropped the "raw" from the topic during RoboCup 2026) laid
out exactly like NUbots' `input::K1Camera` `SharedImageHeader` **including the leading magic/version fields
robocup2026 added** — a layout mismatch shifts the interprocess mutex offset and aborts the reader with a
glibc `pthread_mutex_lock` owner assertion on the first frame.
So the sim impersonates NUbridge: the **unchanged** NUbots `robocup`/`behaviour` role reads the segment →
`ImageCompressor` → `NetworkForwarder` → **NUsight** shows `CompressedImage`, same as on the real robot.
`--ipc host` (already used by `./b run` and NUbots' `./b run`) shares `/dev/shm` across the containers.
Config: `mujoco/config/camera.yaml` (segment name, resolution, fps, intrinsics). Renders via **EGL**
(offscreen, no window), so it works **headless** too — just needs a render device (`./b run` passes
`/dev/dri` + GPU). No device ⇒ logs and disables, no crash — but then vision receives **zero** frames
(`VisualMesh Stats: Receiving 0/s`): confirm the sim log shows `Camera: rendering 640 x 480 ...` before
blaming the NUbots side. The right-camera segment (`_boostercamera_head_raw_right_rgb`) is not rendered
yet; K1Camera warn-retries on it harmlessly (stereo is future work).

The same render thread also publishes the **head-pose segment** (`_head_pose`, K1Sensors' "NBPO" layout,
`pose_segment:` in `camera.yaml`): the `Head_2` pose in the yaw-only base footprint frame, exactly what
NUbridge publishes on the real robot. This matters more than it looks: NUbots' odometry is yaw-only, so
`Sensors.Htw` gets its pitch/roll **only** from this pose — without it `GetUpPlanner` never sees the robot
as fallen and the whole FallRecovery → GetUpPlanner → `K1GetUpPolicy` chain stays dead. The NUbots-side
walk policy (`K1WalkPolicy`) also masks the head joints out of its observation: the policy trained with a
pinned head, and real K1Look scan amplitudes push the raw observation out of distribution and topple the
robot within seconds.

## 7. Getting a locomotion policy

NUSim neither trains nor runs policies — it only simulates. Policies are trained in the NUbots
**[mujoco_playground fork](https://github.com/Tom0Brien/mujoco_playground)** (branch `feat/k1-training`,
MJX/brax PPO: `K1JoystickFlatTerrain` / `K1JoystickRoughTerrain` for walking, `K1Getup` for fall
recovery) — `learning/train_jax_ppo.py --env_name=<task> --domain_randomization`, then export a checkpoint
with `learning/export_k1_onnx.py` (bakes the brax observation normalization into the graph). The exported
`.onnx` is **deployed into NUbots_K1** (`module/skill/K1WalkPolicy/data/k1_walk.onnx`,
`module/skill/K1GetUpPolicy/data/k1_getup.onnx`), where it runs against this sim (or the real robot)
through CUSTOM mode + `rt/joint_ctrl`. The walk interface is pinned in
**[OBS_ACTION_CONTRACT.md](OBS_ACTION_CONTRACT.md)**.

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
