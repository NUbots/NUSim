# Dropping a trained policy into the sim

Once you have a trained policy exported per `../export/export_onnx.py` (fixed `obs`/`action` I/O names,
matching `../OBS_ACTION_CONTRACT.md`):

1. **Put the file here**, e.g. `mujoco/training/models/k1_policy.onnx` (this directory is a reasonable
   default location; any path works as long as the config below points at it).

2. **Point `config/locomotion.yaml` at it and select the policy backend:**

   ```yaml
   backend: policy # was: kinematic

   policy:
     path: "training/models/k1_policy.onnx" # relative to mujoco/, like `model:` in simulation.yaml
     action_scale: 0.25 # must match what the policy was trained with
     inference_divisor: 20 # 1 kHz / 20 = 50 Hz -- must match training's control decimation
   ```

3. **Build with the ONNX runtime backend enabled** (off by default, since it pulls in onnxruntime):

   ```bash
   K1SIM_CMAKE_ARGS="-DK1_WITH_ONNX=ON" ./docker/k1sim.sh build
   ```

4. Run as usual (`scripts/k1/run_mujoco.sh` or `./docker/k1sim.sh run`). `ChangeMode(kWalking)` /
   `ChangeMode(kSoccer)` now drive the robot through the loaded policy instead of the kinematic servo.

## Sanity-checking a policy before dropping it in

- Confirm the I/O contract with `onnxruntime` directly:

  ```python
  import onnxruntime as ort
  sess = ort.InferenceSession("k1_policy.onnx")
  print([(i.name, i.shape) for i in sess.get_inputs()])   # [('obs', [1, 75])]
  print([(o.name, o.shape) for o in sess.get_outputs()])  # [('action', [1, 22])]
  ```

- `mujoco/test/unit/assets/random_policy.onnx` is a tiny untrained MLP used by the sim's own unit tests to
  smoke-test the ONNX plumbing (loads, runs, produces a finite 22-vector) — useful as a reference for the
  exact shapes/names expected, not as a starting point for training.

## No policy ships by default

`config/locomotion.yaml`'s `backend` defaults to `kinematic` and `policy.path` is empty — the sim runs fully
functional without ever needing this directory. The policy backend is strictly opt-in.
