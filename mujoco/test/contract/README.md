# Contract tests

Two independent test rigs live here:

- `check_model.py` — model/scene acceptance (workstream A, M1); see its docstring.
- `test_sdk_roundtrip.py` + `host_client/` — the **M4/M5 wire-compat contract test**
  (workstream C), documented below.

## What `test_sdk_roundtrip.py` proves

That `module::SdkBridge` is **byte-level compatible** with the Booster SDK wire
protocol: registered DDS type names, CDR encoding of every published type, and the
JSON RPC request/response scheme. It does this the only way that actually proves
it — by driving the sim with **the real Booster SDK as a client** (`B1LocoClient`
+ `ChannelSubscriber`), the same code path NUbots_K1's `HardwareIO` uses. The SDK
is used strictly as a *test client*; nothing from it is linked into `k1_mujoco_sim`
(the sim's types are clean-room IDL, see `mujoco/idl/` and
`module/SdkBridge/PROTOCOL.md`).

Checks (each printed as `[PASS]`/`[FAIL]`, exit code 0 iff all pass):

| # | Check |
|---|---|
| 1 | `rt/low_state` received at ≥ 45 msgs/s over a 5 s window |
| 2 | every sample's `motor_state_serial` has exactly 22 entries |
| 3 | motor/IMU values finite and plausible magnitude |
| 4 | `rt/odometer_state` received |
| 5 | `rt/battery_state` received at 1 Hz with soc ∈ (0,100] (pinned-SDK client build only, see below) |
| 6 | `ChangeMode(kPrepare)` returns 0 within 1000 ms |
| 7 | `GetMode()` returns 0 and reports `kPrepare` (exercises the RPC *response-body* path) |
| 8 | `Move(0.1,0,0)` returns 0 within 1000 ms |
| 9 | `RotateHead(0.2,0.3)` returns 0 within 1000 ms |
| 10 | `GetUp()` returns 0 within 1000 ms |

## Client build strategy (what was chosen and why)

`host_client/sdk_client_check.cpp` is a small C++ client compiled **on the host**
(g++ 11, not inside the k1sim docker image) by `host_client/build.sh`, linking:

- the SDK's prebuilt `lib/x86_64/libbooster_robotics_sdk.a`
- the SDK's bundled `third_party/lib/x86_64/{libfastrtps.so, libfastcdr.so,
  libfoonathan_memory-0.7.3.a}` (SONAME `libfastrtps.so.2.13` — same Fast-DDS 2.13
  line the sim pins at 2.13.6)

This was chosen over the SDK's python bindings because the bindings require a
scikit-build/pybind11 build of the same `.a` anyway (strictly more moving parts,
identical wire coverage), while the C++ client exercises exactly the
`ChannelFactory`/`B1LocoClient` code path NUbots' `HardwareIO` uses.

`build.sh` defaults to the older local checkout at
`/home/nubots/Workspace/booster/booster_robotics_sdk` (commit `7fb7287`); override
with `BOOSTER_SDK_ROOT=<path>` to build against another SDK tree, e.g. an extract
of the pinned `324946e7` tarball. Both were used for the recorded results below —
the two SDK versions are wire-identical for everything this test touches (verified
via `strings` on both `.a`s; see PROTOCOL.md §2). Version quirks the client absorbs
(`#if __has_include(<booster/idl/b1/BatteryState.h>)` detects the pinned SDK):

- **battery_state**: only the pinned SDK ships `BatteryState.h`/compiles the type
  into its `.a`, so check #5 is compiled only in pinned-SDK builds (`[SKIP]`
  otherwise).
- **Init**: the pinned SDK's `ChannelFactory::Init(domain)` with no network
  interface tries to load a `FASTRTPS_DEFAULT_PROFILES_FILE` XML and fails to
  create a participant without one; the client calls its `InitDefault(domain)`
  instead (the older SDK has no `InitDefault` declaration and its `Init(0)` works
  directly). On the real robot NUbots' HardwareIO runs with Booster's environment
  (profiles file/interface provided); irrelevant to the sim side of the wire.

## Running

```bash
# 1. build the sim (once)
cd mujoco && ./docker/k1sim.sh build           # or K1SIM_BUILD_DIR=... ./docker/k1sim.sh build

# 2. run the contract test (builds host_client/ on first run)
./test/contract/test_sdk_roundtrip.py          # add --build-dir NAME if not auto-detected
```

The script starts `k1_mujoco_sim --headless` in docker (`--network host --ipc host
--user $(id -u)`), waits for `SdkBridge ready`, runs the client, prints the
PASS/FAIL summary and stops the container. `--synthetic` swaps in
`sdkbridge_synthetic_sim` (ConsoleLog + SdkBridge + a scripted `SimStateUpdate`
source, no physics — see `module/SdkBridge/test_support/`) for when
`module::Simulation` is stubbed or being bisected.

**Pitfall**: never start the sim container as root (bare `docker run` without
`--user`). It leaves root-owned Fast-DDS SHM segments in the shared `/dev/shm`
which a later uid-1000 participant can neither write to nor delete — host→sim RPC
then times out while some sim→host topics still flow. Details in PROTOCOL.md §4.

## Recorded results (2026-07-08, real `k1_mujoco_sim --headless`, build-wsC)

Both client variants, all checks `PASS`:

| Metric | local SDK 7fb7287 | pinned SDK 324946e7 |
|---|---|---|
| `rt/low_state` rate | 50.00 msgs/s (250/5 s) | 50.00 msgs/s (250/5 s) |
| `motor_state_serial` size | 22 | 22 |
| `rt/odometer_state` | received | received |
| `rt/battery_state` | n/a (type absent in SDK) | 5 samples/5 s, soc=100.0 |
| `ChangeMode(kPrepare)` | 0 in 0.32 ms | 0 in 0.39 ms |
| `GetMode()` | 0, mode=1, 0.59 ms | 0, mode=1, 0.53 ms |
| `Move(0.1,0,0)` | 0 in 0.52 ms | 0 in 0.61 ms |
| `RotateHead(0.2,0.3)` | 0 in 0.50 ms | 0 in 0.42 ms |
| `GetUp()` | 0 in 0.41 ms | 0 in 0.26 ms |

RPC latencies are ~3 orders of magnitude inside `B1LocoClient`'s 1000 ms timeout
(replies are written synchronously inside the sim's DDS listener callback). The
sim's log confirms end-to-end dispatch into `module::Locomotion` (`ChangeMode
requested: mode 1`, `GetUp requested (target mode 4)`; Move/RotateHead set
controller state without logging), and `GetMode()` returning `mode=1` proves the
Move/RotateHead path too — the mode it reads back travelled RPC → Locomotion →
SimStateUpdate → SdkBridge's cache.
