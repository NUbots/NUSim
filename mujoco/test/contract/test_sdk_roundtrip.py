#!/usr/bin/env python3
"""M4/M5 contract test: proves module::SdkBridge is byte-compatible with the real
Booster SDK wire protocol (registered type names + CDR encoding + RPC JSON), by
running the sim in docker and driving it with a client built against the actual
Booster SDK (client only — never linked into the sim). See README.md here.

Usage:
    ./test_sdk_roundtrip.py [--synthetic] [--build-dir NAME] [--keep-sim]

    --synthetic   run the sdkbridge_synthetic_sim test binary instead of the real
                  k1_mujoco_sim (for when module::Simulation is stubbed/broken)
    --build-dir   docker build dir under mujoco/ containing the binaries
                  (default: $K1SIM_BUILD_DIR, else auto-detect)
    --keep-sim    leave the sim container running after the test (debugging)

Exit code 0 iff every check passed.
"""

import argparse
import os
import pathlib
import subprocess
import sys
import time

MUJOCO_DIR = pathlib.Path(__file__).resolve().parents[2]  # .../mujoco
REPO_DIR = MUJOCO_DIR.parent  # .../NUSim
HOST_CLIENT_DIR = pathlib.Path(__file__).resolve().parent / "host_client"
CONTAINER_NAME = "k1sim_contract_roundtrip"
IMAGE = os.environ.get("K1SIM_IMAGE", "k1sim:latest")


def sim_binary(build_dir: str, synthetic: bool) -> str:
    return (
        f"{build_dir}/module/SdkBridge/test_support/sdkbridge_synthetic_sim"
        if synthetic
        else f"{build_dir}/k1_mujoco_sim"
    )


def find_build_dir(explicit: str | None, synthetic: bool) -> str:
    candidates = [explicit] if explicit else []
    if os.environ.get("K1SIM_BUILD_DIR"):
        candidates.append(os.environ["K1SIM_BUILD_DIR"])
    candidates += ["build-docker", "build-wsC"]
    candidates += [p.name for p in MUJOCO_DIR.glob("build-*") if p.is_dir()]
    for name in candidates:
        if name and (MUJOCO_DIR / sim_binary(name, synthetic)).exists():
            return name
    sys.exit(
        f"error: no build dir with {sim_binary('<dir>', synthetic)} found under mujoco/ "
        "(run `K1SIM_BUILD_DIR=<dir> ./docker/k1sim.sh build` first)"
    )


def ensure_client() -> pathlib.Path:
    client = HOST_CLIENT_DIR / "sdk_client_check"
    src = HOST_CLIENT_DIR / "sdk_client_check.cpp"
    if not client.exists() or client.stat().st_mtime < src.stat().st_mtime:
        print("[roundtrip] building host SDK client ...", flush=True)
        subprocess.run([str(HOST_CLIENT_DIR / "build.sh")], check=True)
    return client


def docker(*args: str, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], **kwargs)


def start_sim(build_dir: str, synthetic: bool) -> None:
    binary = "./" + sim_binary(build_dir, synthetic)
    cmd = [
        "run", "-d", "--rm",
        "--name", CONTAINER_NAME,
        "--network", "host", "--ipc", "host",
        # CRITICAL: run as the invoking user. A root-run sim leaves root-owned
        # Fast-DDS SHM segments in the shared /dev/shm that the host client
        # can't write to -> host->sim RPC silently times out. See PROTOCOL.md §4.
        "--user", f"{os.getuid()}:{os.getgid()}",
        "-v", f"{REPO_DIR}:/workspace/NUSim",
        "-w", "/workspace/NUSim/mujoco",
        IMAGE,
    ] + [binary] + ([] if synthetic else ["--headless"])
    print(f"[roundtrip] starting sim: {binary}" + (" (synthetic state source)" if synthetic else ""), flush=True)
    docker(*cmd, check=True, stdout=subprocess.DEVNULL)

    # Wait for the DDS surface to come up.
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        logs = docker("logs", CONTAINER_NAME, capture_output=True, text=True).stdout
        if "SdkBridge ready" in logs:
            print("[roundtrip] sim is up", flush=True)
            return
        if docker("inspect", CONTAINER_NAME, capture_output=True).returncode != 0:
            sys.exit("error: sim container exited during startup:\n" + logs)
        time.sleep(0.5)
    sys.exit("error: sim did not report 'SdkBridge ready' within 30 s")


def stop_sim() -> None:
    docker("stop", CONTAINER_NAME, capture_output=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--synthetic", action="store_true")
    parser.add_argument("--build-dir", default=None)
    parser.add_argument("--keep-sim", action="store_true")
    args = parser.parse_args()

    build_dir = find_build_dir(args.build_dir, args.synthetic)
    client = ensure_client()

    stop_sim()  # clear any leftover container from an aborted run
    start_sim(build_dir, args.synthetic)
    try:
        # Client prints its own [PASS]/[FAIL] lines and a summary.
        result = subprocess.run([str(client)], timeout=120)
        code = result.returncode
    except subprocess.TimeoutExpired:
        print("[roundtrip] FAIL: client did not finish within 120 s", flush=True)
        code = 2
    finally:
        if args.keep_sim:
            print(f"[roundtrip] leaving sim container '{CONTAINER_NAME}' running (--keep-sim)", flush=True)
        else:
            stop_sim()

    print(f"[roundtrip] {'PASS' if code == 0 else 'FAIL'} (client exit code {code})", flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
