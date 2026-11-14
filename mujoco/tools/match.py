"""./b match — run a refereed match: the sim, plus a hidden NUbots process for every robot.

    ./b match --game 3                              3 a side, nusim/behaviour on every robot
    ./b match --game 1 --away-role <role>           a different stack for each team
    ./b match --game 2 --headless --rtf 1           no window; the game starts by itself

NUbots_K1 must be built first (./b target generic, ./b build there). Each robot runs in its own
container, with its output in mujoco/<build dir>/match/robot<id>/log. Stock NUbots joins DDS domain 0
and reads one camera, so each container gets files that point it at its own robot:

  * a FastDDS profile whose portBase is the one for the robot's domain. The SDK ignores a domainId
    in the profile, but ports are all that separates domains on the wire.
  * a NUSimCamera.yaml naming the robot's segment (see MatchCameras::segment).

Every container also gets a FieldDescription.yaml for the sim's field: NUbots defaults to its 6.8 x 5 m
lab field, and matching lines against the wrong field leaves robots lost.
"""

import getpass
import json
import math
import os
import pathlib
import re
import signal
import subprocess
import sys
import time

from _util import build_dir

MUJOCO = pathlib.Path(__file__).resolve().parent.parent
PORT_BASE = 7400  # RTPS defaults: a domain's ports start at PB + DG * domain
DOMAIN_GAIN = 250
CONTAINER_BUILD = "/home/booster/build"  # where NUbots' build volume mounts in its image
SIM_STARTUP_SECONDS = 3
SIM_CONTAINER = "nusim-match-sim"

PROFILE = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="booster_dds" is_default_profile="true">
        <rtps>
            <name>booster_dds</name>
            <port><portBase>{port_base}</portBase></port>
        </rtps>
    </participant>
</profiles>
"""

CAMERA = """log_level: INFO
cameras:
  - segment: {segment}
    name: Left Camera
    id: 0
"""


# NUbots' FieldDescription.yaml preset keys, and where match.yaml's field has each value.
# Read by NUbots' input::NUSimGroundTruth, which takes NUSim's rt/nusim/gt topics in place of vision + localisation.
GROUND_TRUTH = """log_level: {log_level}
ground_truth: true
robot_topic: rt/nusim/gt/robot
ball_topic: rt/nusim/gt/ball
world_to_field_yaw: {yaw}
timeout: 0.5
"""

MODES = ["game", "testing", "freeplay"]
FIELDS = ["middle", "kidsize", "no-field"]  # simulation.yaml's fields; matches always use the middle field
GAMECONTROLLER_DIR = ".deps/GameController"  # the checkout module::Referee loads its library and rules from

# What a team can run. A codebase needs a way to start one player per robot; only NUbots has one so far.
CODEBASES = [
    dict(name="NUbots", available=True, note=""),
    dict(name="rUNSWift", available=False,
         note="rUNSWift isn't set up on this machine yet: tools/match.py needs to know how to start its player."),
]
# The NUbots role a refereed game runs: the competition role with the sim's walk, kick and camera.
NUBOTS_GAME_ROLES = ["nusim/robocup", "nusim/behaviour"]

FIELD_PRESET = {
    "FieldLength": "length",
    "FieldWidth": "width",
    "GoalWidth": "goal_width",
    "GoalCrossbarHeight": "goal_height",
    "GoalAreaLength": "goal_area_length",
    "GoalAreaWidth": "goal_area_width",
    "PenaltyMarkDistance": "penalty_mark_distance",
    "PenaltyAreaLength": "penalty_area_length",
    "PenaltyAreaWidth": "penalty_area_width",
}


def register(command):
    command.description = "Run a match: the sim with a hidden NUbots process per robot"
    command.add_argument("--game", type=int, default=1, choices=[1, 2, 3], help="robots a side")
    command.add_argument("--home-role", help="testing: NUbots role for the first team (default: the game role)")
    command.add_argument("--away-role", help="testing: NUbots role for the second team (default: the game role)")
    command.add_argument("--nubots-target", default="generic", help="the NUbots_K1 ./b target that was built")
    command.add_argument("--mode", default="game", choices=MODES, help="game: automatic referee; testing: you call "
                         "the set plays; freeplay: no referee")
    command.add_argument("--ground-truth", action="store_true",
                         help="robots take their pose and the ball from the sim instead of vision + localisation "
                         "(the default in game mode)")
    command.add_argument("--vision", action="store_true", help="game mode: use vision + localisation after all")
    command.add_argument("--vision-device", default="auto", choices=["auto", "gpu", "cpu"],
                         help="where NUbots' Yolo runs. auto uses the GPU unless NUbots' image can't drive it")
    command.add_argument("--replace", action="store_true", help="stop a ./b match that is already running first")
    command.add_argument("--headless", action="store_true", help="no window; the game starts by itself")
    command.add_argument("--rtf", help="real-time factor")


def available_roles(nubots_target):
    """The roles in the NUbots build, for the window's team pickers."""
    user = getpass.getuser()
    found = subprocess.run(
        ["docker", "run", "--rm", "--entrypoint", "find",
         "--volume", f"nubots_{nubots_target}_{user}_build:{CONTAINER_BUILD}",
         f"{user}/nubots:{nubots_target}", f"{CONTAINER_BUILD}/bin", "-maxdepth", "2", "-type", "f", "-executable",
         "-not", "-path", "*/lib/*"],
        capture_output=True, text=True,
    ).stdout  # fmt: skip
    return sorted(line.removeprefix(f"{CONTAINER_BUILD}/bin/") for line in found.split())


def yolo_config(choice, nubots_target, session):
    """Where Yolo should run, and the Yolo.yaml to mount for it (None keeps NUbots' own, on the GPU).

    NUbots runs Yolo through TensorRT. Its image has TensorRT 10.3, which predates the RTX 50 series
    and fails on it ("Unsupported SM: 0xc00"), leaving the robots blind. Those GPUs fall back to the CPU.
    """
    if choice == "auto":
        gpu = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                             capture_output=True, text=True).stdout
        choice = "cpu" if re.search(r"RTX 50\d\d", gpu) else "gpu"
        if choice == "cpu":
            print(f"Vision: {gpu.strip()} is too new for the TensorRT in NUbots' image, so Yolo runs on the CPU "
                  "(--vision-device gpu overrides)", flush=True)
    if choice == "gpu":
        return None
    user = getpass.getuser()
    stock = subprocess.run(
        ["docker", "run", "--rm", "--entrypoint", "cat",
         "--volume", f"nubots_{nubots_target}_{user}_build:{CONTAINER_BUILD}",
         f"{user}/nubots:{nubots_target}", f"{CONTAINER_BUILD}/config/Yolo.yaml"],
        capture_output=True, text=True, check=True,
    ).stdout  # fmt: skip
    path = session / "Yolo.yaml"
    path.write_text(re.sub(r"^device:.*$", "device: cpu", stock, flags=re.M))
    return path


def config_value(file, key):
    """A top-level scalar from a config file, which needn't parse the rest of it."""
    return re.search(rf"^{key}:\s*(\S+)", (MUJOCO / "config" / file).read_text(), re.M).group(1)


def field_value(key):
    return float(re.search(rf"^  {key}:\s*(\S+)", (MUJOCO / "config/match.yaml").read_text(), re.M).group(1))


def field_description(nubots_target):
    """NUbots' FieldDescription.yaml, changed to describe the sim's field."""
    user = getpass.getuser()
    stock = subprocess.run(
        ["docker", "run", "--rm", "--entrypoint", "cat",
         "--volume", f"nubots_{nubots_target}_{user}_build:{CONTAINER_BUILD}",
         f"{user}/nubots:{nubots_target}", f"{CONTAINER_BUILD}/config/FieldDescription.yaml"],
        capture_output=True, text=True, check=True,
    ).stdout  # fmt: skip

    preset = "".join(f"    {name}: {field_value(key)}\n" for name, key in FIELD_PRESET.items())
    preset += f"    CenterCircleDiameter: {2 * field_value('center_circle_radius')}\n    GoalDepth: 0.6\n"
    text = re.sub(r"^field_type:.*$", "field_type: nusim", stock, flags=re.M)
    text = re.sub(r"^BallRadius:.*$", f"BallRadius: {field_value('ball_radius')}", text, flags=re.M)
    text = re.sub(r"^LineWidth:.*$", f"LineWidth: {field_value('line_width')}", text, flags=re.M)
    return text.replace("presets:\n", f"presets:\n  nusim:\n{preset}", 1)


def roster(game):
    """The default roster, as config::robot_roster() builds it."""
    first_domain = int(config_value("dds.yaml", "domain"))
    return [
        dict(robot_id=i + 1, team_id=125 + i // game, player_id=i % game + 1, domain=first_domain + i)
        for i in range(2 * game)
    ]


def gpu_flags():
    """GPU access for NUbots' vision, found the same way as docker/k1sim.sh's."""
    if list(pathlib.Path("/etc/cdi").glob("nvidia*.yaml")) + list(pathlib.Path("/var/run/cdi").glob("nvidia*.yaml")):
        return ["--device", "nvidia.com/gpu=all", "-e", "NVIDIA_DRIVER_CAPABILITIES=all"]
    runtimes = subprocess.run(["docker", "info", "--format", "{{json .Runtimes}}"], capture_output=True, text=True)
    return ["--gpus", "all", "-e", "NVIDIA_DRIVER_CAPABILITIES=all"] if '"nvidia"' in runtimes.stdout else []


def start_robot(robot, role, session, nubots_target, ground_truth, first_team):
    directory = session / f"robot{robot['robot_id']}"
    directory.mkdir(parents=True, exist_ok=True)

    base = config_value("camera.yaml", "segment")
    segment = base if robot["robot_id"] == 1 else f"{base}_robot{robot['robot_id']}"
    (directory / "fastdds.xml").write_text(PROFILE.format(port_base=PORT_BASE + DOMAIN_GAIN * robot["domain"]))
    (directory / "NUSimCamera.yaml").write_text(CAMERA.format(segment=segment))
    # NUbots' field frame has its own goal on +x. The first team defends the sim's -x goal.
    yaw = math.pi if first_team else 0.0
    (directory / "NUSimGroundTruth.yaml").write_text(GROUND_TRUTH.format(yaw=yaw, log_level=os.environ.get("K1SIM_GROUND_TRUTH_LOG", "INFO")))
    truth_config = f"{CONTAINER_BUILD}/config/NUSimGroundTruth.yaml"
    truth_mount = ["--volume", f"{directory}/NUSimGroundTruth.yaml:{truth_config}:ro"] if ground_truth else []

    user = getpass.getuser()
    command = [
        "docker", "run", "--rm",
        "--name", container(robot),
        "--network", "host",
        "--ipc", "host",
        "--user", f"{os.getuid()}:{os.getgid()}",
        "--workdir", CONTAINER_BUILD,
        "--env", "FASTRTPS_DEFAULT_PROFILES_FILE=/robot/fastdds.xml",
        "--volume", f"nubots_{nubots_target}_{user}_build:{CONTAINER_BUILD}",
        "--volume", f"{directory}:/robot:ro",
        "--volume", f"{directory}/NUSimCamera.yaml:{CONTAINER_BUILD}/config/NUSimCamera.yaml:ro",
        "--volume", f"{session}/FieldDescription.yaml:{CONTAINER_BUILD}/config/FieldDescription.yaml:ro",
        *truth_mount,
        *(["--volume", f"{session}/Yolo.yaml:{CONTAINER_BUILD}/config/Yolo.yaml:ro"]
          if (session / "Yolo.yaml").exists() else []),
        # Extra config overrides for every robot, e.g. a module's yaml with log_level: DEBUG:
        # K1SIM_ROBOT_MOUNTS="/abs/FieldPlayer.yaml:config/FieldPlayer.yaml,..."
        *(flag for mount in filter(None, os.environ.get("K1SIM_ROBOT_MOUNTS", "").split(","))
          for flag in ("--volume", "{}:{}/{}:ro".format(mount.split(":")[0], CONTAINER_BUILD, mount.split(":")[1]))),
        *gpu_flags(),
        "--entrypoint", f"bin/{role}",
        f"{user}/nubots:{nubots_target}",
        "--player_id", str(robot["player_id"]),
        "--team_id", str(robot["team_id"]),
    ]  # fmt: skip
    log = (directory / "log").open("w")
    return subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)


def container(robot):
    return f"nusim-match-robot{robot['robot_id']}"


def stop_containers(robots):
    names = [SIM_CONTAINER, *(container(robot) for robot in robots)]
    subprocess.run(["docker", "rm", "--force", *names], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def run_once(setup, session, nubots_target, headless, rtf):
    """One sim session with its robots. Returns the window's next setup, or None once the sim just exits."""
    mode = setup["mode"]
    stop_containers(roster(3))  # from a run that didn't finish, whatever its size

    request = session / "request.json"
    request.unlink(missing_ok=True)
    (session / "session.json").write_text(json.dumps(setup, indent=2))

    run_flags = f"--name {SIM_CONTAINER}"
    game_role = next((role for role in NUBOTS_GAME_ROLES if role in setup["roles"]), "nusim/behaviour")
    if mode == "game":
        # The automatic referee. Players that obey the GameController mustn't be held still between phases.
        role = "bin/sim/match"
        # Robots start on the touchlines and walk on when the game is Ready.
        sim_args = ["--game", str(setup["game"])]
        sim_args += ["--free-robots"] if game_role == "nusim/robocup" else []
        roles = (game_role, game_role)
    elif mode == "testing":
        # The built-in GameController, operated by hand from the window (or the browser viewer).
        role = "bin/sim/testing"
        # Robots start on the touchlines here too, and walk on when you call Ready.
        sim_args = ["--game", str(setup["game"]), "--referee"]
        run_flags += f" -e GAMECONTROLLER_DIR={GAMECONTROLLER_DIR}"
        roles = (setup["home_role"], setup["away_role"])
    else:
        # Just the sim: one robot on the chosen field, for a role started by hand from a terminal.
        role = "bin/sim/soccer"
        # --gamecontroller only listens: a GameController on the network is obeyed if there is one.
        sim_args = ["--field", setup["field"], "--gamecontroller"]
        sim_args += ["--robots", str(setup["robots"])] if setup["robots"] > 1 else []
        roles = None
    sim_args += ["--session", f"{build_dir()}/match"]
    sim_args += ["--headless"] if headless else []
    sim_args += ["--rtf", rtf] if rtf else []

    robots = roster(setup["game"]) if roles else []
    sim = subprocess.Popen([str(MUJOCO / "docker/k1sim.sh"), "run", role, *sim_args],
                           env=dict(os.environ, K1SIM_RUN_FLAGS=run_flags))

    try:
        time.sleep(SIM_STARTUP_SECONDS)
        processes = {}
        for robot in robots:
            first_team = robot["team_id"] == robots[0]["team_id"]
            processes[robot["robot_id"]] = start_robot(robot, roles[0] if first_team else roles[1], session,
                                                       nubots_target, setup["ground_truth"], first_team)
        if robots:
            print(f"{len(robots)} robots starting ({roles[0]} v {roles[1]}). Logs are in {session}/robot<id>/log",
                  flush=True)
        else:
            print("Free Play: no robot code started. Run a role yourself, e.g. ./b run nusim/behaviour", flush=True)

        # Their terminals are hidden, so say when one stops. Roles that need a keyboard can't run here.
        while sim.poll() is None:
            for robot_id, process in list(processes.items()):
                if process.poll() is not None:
                    print(f"WARNING: robot {robot_id} stopped, see {session}/robot{robot_id}/log", flush=True)
                    del processes[robot_id]
            time.sleep(1)
    finally:
        stop_containers(roster(3))

    # The window's "Apply and Restart" leaves the setup it wants next, then closes the sim.
    if not request.exists():
        return None
    wanted = json.loads(request.read_text())
    request.unlink()
    for key in ("game", "mode", "home_role", "away_role", "home_codebase", "away_codebase", "ground_truth", "field",
                "robots"):
        setup[key] = wanted.get(key, setup[key])
    setup["game"] = min(3, max(1, int(setup["game"])))
    setup["robots"] = min(6, max(1, int(setup["robots"])))
    setup["mode"] = setup["mode"] if setup["mode"] in MODES else "game"
    setup["field"] = setup["field"] if setup["field"] in FIELDS else FIELDS[0]
    available = [codebase["name"] for codebase in CODEBASES if codebase["available"]]
    for key in ("home_codebase", "away_codebase"):
        if setup[key] not in available:
            print(f"WARNING: {setup[key]} can't be started here, using {available[0]}", flush=True)
            setup[key] = available[0]
    return setup


def run(game, home_role, away_role, nubots_target, mode, ground_truth, vision, vision_device, replace, headless,
        rtf, **kwargs):
    # Only one match fits on a machine: the containers, DDS domains and camera segments are shared names.
    # Starting a second would silently kill the first one's sim and robots.
    running = subprocess.run(["docker", "ps", "--quiet", "--filter", f"name=^{SIM_CONTAINER}$"],
                             capture_output=True, text=True).stdout.strip()
    if running and not replace:
        sys.exit("error: a ./b match is already running on this machine. Close its window first, or pass --replace")

    if not (MUJOCO / build_dir() / "bin/sim/match").exists():
        sys.exit("error: no bin/sim/match. Run tools/install_game_controller_api.sh, ./b configure and ./b build")

    session = MUJOCO / build_dir() / "match"
    session.mkdir(parents=True, exist_ok=True)
    (session / "FieldDescription.yaml").write_text(field_description(nubots_target))
    (session / "Yolo.yaml").unlink(missing_ok=True)
    yolo_config(vision_device, nubots_target, session)

    # Closing the terminal or killing this script mustn't leave containers behind.
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)

    roles = available_roles(nubots_target)
    game_role = next((role for role in NUBOTS_GAME_ROLES if role in roles), "nusim/behaviour")
    setup = dict(game=game, mode=mode, home_role=home_role or game_role, away_role=away_role or game_role,
                 ground_truth=ground_truth or (mode == "game" and not vision),
                 home_codebase="NUbots", away_codebase="NUbots", field=FIELDS[0], robots=1,
                 roles=roles, codebases=CODEBASES, fields=FIELDS)
    try:
        while setup is not None:
            setup = run_once(setup, session, nubots_target, headless, rtf)
    except KeyboardInterrupt:
        pass
    finally:
        stop_containers(roster(3))
