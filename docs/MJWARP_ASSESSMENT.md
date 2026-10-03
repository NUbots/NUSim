# MuJoCo or MJWarp for NUSim?

Assessment date: 3 October 2026. Keep native MuJoCo for the interactive NUSim application on macOS.
Consider MJWarp separately for batched training or evaluation on an NVIDIA GPU.

[MuJoCo's guidance](https://mujoco.readthedocs.io/en/stable/mjwarp/index.html#when-to-use-mjwarp)
distinguishes MuJoCo's low step latency from MJWarp's throughput across many simulations.
[Warp's macOS builds](https://github.com/NVIDIA/warp#installing) support CPU execution, with no Metal
acceleration. [MJWarp](https://github.com/google-deepmind/mujoco_warp#getting-started) uses its CPU path
for development/debugging and needs an NVIDIA GPU for fast simulation.

## Local check

On an Apple M5 Pro MacBook Pro (15 CPU cores, 48 GB RAM, macOS 26.7.1), the unmodified Middle Division
scene compiled, converted to MJWarp and stepped on CPU. Versions: MuJoCo 3.10.0, MJWarp 3.10.0.3,
Warp 1.17.0, Python 3.11. Warp reported only the `cpu` device.

The scene has 34 velocity DoFs, 155 geoms and a 1 ms timestep. Starting from `ready`, each backend
ran 100 warm-up steps followed by three groups of 100 steps with zero actuator commands. Median
wall time per step, including the Python call, was:

| Backend | Time per physics step |
| --- | --- |
| Native MuJoCo (`mj_step`) | 0.017 ms |
| MJWarp CPU (`step`) | 2.89 ms |

MJWarp CPU was approximately 170 times slower in this small check. This measures one world without
DDS, camera rendering, policy inference or the simulator's controller. It establishes model conversion
and stepping, not equivalent trajectories or contact fidelity. It is not a CUDA benchmark and does
not predict batched NVIDIA performance. Timing varies with machine load and the contact state.

## Fit with the existing application

NUSim advances one world at 1 kHz through the C++ API. `--robots` adds bodies to that same world;
it does not create batches of independent environments. NUClear modules access `mjModel`/`mjData`
directly for joint control, sensors, rendering, resets and supervisor placement.

MJWarp supplies Python APIs and device arrays. Adopting it would require a new physics backend and
state-transfer boundary for those modules, with synchronisation/readback for DDS and the existing
renderer. It would not solve the macOS camera/shared-memory boundary or supply a walking policy.
