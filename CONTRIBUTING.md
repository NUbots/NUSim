# Contributing

Base focused contributions on [NUbots/NUSim](https://github.com/NUbots/NUSim)'s `main` branch. Keep changes
to walking policies in their own repositories; NUSim supplies physics and the Booster SDK interface.

For a fork with an `origin` remote, add upstream once and fetch its branch references:

```bash
git remote add upstream https://github.com/NUbots/NUSim.git
git fetch upstream
git switch -c macos-native-support upstream/main
```

Fetching upstream makes its other branches available without merging them. Inspect a feature with
`git log upstream/main..upstream/<branch>` or use a separate checkout when testing it. Keep unrelated
features out of the contribution branch.

Build using the [setup guide](docs/K1_MUJOCO_SETUP.md):

```bash
./b configure
./b build
./b test
./b format
./b format --check
git diff --check
```

The native macOS tests include offscreen rendering on a worker thread and shared-memory image
publication. The DDS test verifies discovery, message delivery and teardown with UDP and UDP+SHM.
Linux uses the existing Docker toolchain by default. To run the additional Python model
checks without changing the host tooling environment:

```bash
uv run --no-project --python 3.11 --with mujoco==3.10.0 --with numpy --with pyyaml \
    python mujoco/test/contract/check_model.py
```

Run the real Booster SDK contract tests in a compatible Linux SDK environment when changing DDS
types, QoS, RPC behaviour or serialisation; see [the contract test guide](mujoco/test/contract/README.md).
Generated IDL code and build/dependency directories must stay out of commits. Preserve the version
pins and wire-format flags unless the contribution explicitly changes those contracts.

Describe the problem, resulting behaviour, tested platforms and outstanding limitations in the pull
request. For macOS, distinguish native simulator validation from an end-to-end NUbots/Booster SDK test.
Push the contribution branch to your fork and open a pull request targeting `NUbots/NUSim:main` after
reviewing the diff and validation results.
