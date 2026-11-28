"""./b match — run a refereed match: the sim, plus a hidden player process (NUbots, B-Human or HTWK) for every robot.

    ./b match --game 3                              3 a side, nusim/behaviour on every robot
    ./b match --game 1 --away-role <role>           a different stack for each team
    ./b match --game 1 --away-codebase B-Human      NUbots v B-Human
    ./b match --game 1 --away-codebase HTWK         NUbots v HTWK
    ./b match --game 2 --headless --rtf 1           no window; the game starts by itself
    ./b match --game 2 --headless --web             no window; watch and run it all from the browser

NUbots must be built first (./b target generic, ./b build there). Each robot runs in its own
container, with its output in mujoco/<build dir>/match/robot<id>/log. Stock NUbots joins DDS domain 0
and reads one camera, so each container gets files that point it at its own robot:

  * a FastDDS profile whose portBase is the one for the robot's domain. The SDK ignores a domainId
    in the profile, but ports are all that separates domains on the wire.
  * a NUSimCamera.yaml naming the robot's segment (see MatchCameras::segment).

Every container also gets a FieldDescription.yaml for the sim's field: NUbots defaults to its 6.8 x 5 m
lab field, and matching lines against the wrong field leaves robots lost. And a RobotCommunication.yaml
that sends team messages to the match network's broadcast address.

The sim and the robots live on their own docker network (NETWORK, SUBNET), one address each, so the GameController's
broadcasts and the team messages never leave it. Robots share the sim's IPC namespace for its camera segments. The
browser viewer's port is the only one published, on 127.0.0.1 unless --web-host says otherwise.

Other teams' code (B-Human, HTWK) runs from each team's image, built by ./b team install from its fork: every robot's
container gets the same environment (docs/MATCH_WINDOW.md#codebases), and the fork's own nusim/run.sh turns it into the settings
its code reads. A team whose motion runs once per LowState at 500 Hz (B-Human) gets it from a copy of config/ with
state_publish_divisor 2, and the roster gives every other robot state_every 10 (still 50 Hz).
"""

import getpass
import ipaddress
import json
import math
import os
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import time

from _util import backend, build_dir, config_section

MUJOCO = pathlib.Path(__file__).resolve().parent.parent
PORT_BASE = 7400  # RTPS defaults: a domain's ports start at PB + DG * domain
DOMAIN_GAIN = 250
# config/match.yaml: its docker: (the match network and containers) and teams: (the codebases a team can play)
DOCKER = config_section("match.yaml", "docker")
TEAMS = config_section("match.yaml", "teams")
CONTAINER_BUILD = TEAMS["NUbots"]["build"]
SIM_STARTUP_SECONDS = DOCKER["sim_startup_seconds"]
SIM_CONTAINER = DOCKER["sim_container"]
# The match's own docker network. The sim and every robot get an address on it, and broadcasts (the GameController's
# control messages, team messages) stay on it instead of going out on this machine's network.
NETWORK = DOCKER["network"]
SUBNET = DOCKER["subnet"]
ROBOT_CPU_SHARES = DOCKER["robot_cpu_shares"]

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
# Used only when the NUbots build has no NUSimGroundTruth.yaml of its own to start from (see ground_truth_config).
GROUND_TRUTH = """log_level: INFO
ground_truth: false
robot_topic: rt/nusim/gt/robot
ball_topic: rt/nusim/gt/ball
world_to_field_yaw: 0.0
timeout: 0.5
warning_period: 5.0
"""

TEAM_NUMBERS = {name: team["number"] for name, team in TEAMS.items()}


def team_image(name):
    """Another team's image: match.yaml's, or K1SIM_<INSTALL>_IMAGE's."""
    team = TEAMS[name]
    return os.environ.get(f"K1SIM_{team['install'].upper()}_IMAGE", team["image"])


MODES = ["game", "testing", "freeplay"]
FIELDS = ["middle", "kidsize", "no-field"]  # simulation.yaml's fields; matches always use the middle field

# What a team can run: NUbots and Nobody always, another team's code once its image is built, rUNSWift not yet.
CODEBASES = [
    dict(
        name=name,
        available=name in ("NUbots", "Nobody"),
        note=(
            f"{name} isn't set up on this machine: build its image ({team_image(name)}) with "
            f"./b team install {team['install']}."
            if "install" in team
            else f"{name} isn't set up on this machine yet: tools/match.py needs to know how to start its player."
        ),
    )
    for name, team in TEAMS.items()
]
NUBOTS_GAME_ROLES = TEAMS["NUbots"]["game_roles"]

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
    names = ", ".join(TEAMS)
    command.add_argument("--home-codebase", default="NUbots", help=f"codebase of the first team ({names})")
    command.add_argument("--away-codebase", default="NUbots", help=f"codebase of the second team ({names})")
    command.add_argument(
        "--home-players", help="the first team's player numbers, comma separated (default 1 to --game)"
    )
    command.add_argument(
        "--away-players", help="the second team's player numbers, comma separated (default 1 to --game)"
    )
    command.add_argument("--nubots-target", default="generic", help="the NUbots ./b target that was built")
    command.add_argument(
        "--mode",
        default="game",
        choices=MODES,
        help="game: automatic referee; testing: you call " "the set plays; freeplay: no referee",
    )
    command.add_argument(
        "--ground-truth",
        action="store_true",
        help="robots take their pose and the ball from the sim instead of vision + localisation "
        "(the default in game mode)",
    )
    command.add_argument("--vision", action="store_true", help="game mode: use vision + localisation after all")
    command.add_argument(
        "--vision-device",
        default="auto",
        choices=["auto", "gpu", "cpu"],
        help="where NUbots' Yolo runs. auto uses the GPU unless NUbots' image can't drive it",
    )
    command.add_argument("--replace", action="store_true", help="stop a ./b match that is already running first")
    command.add_argument(
        "--headless",
        action="store_true",
        help="no window; the game starts by itself, unless --web is there to start it from",
    )
    command.add_argument("--rtf", help="real-time factor")
    command.add_argument(
        "--web",
        action="store_true",
        help="also serve the browser viewer (http://127.0.0.1:8080/), which has the window's " "panels and controls",
    )
    command.add_argument("--web-host", help="address the browser viewer listens on (0.0.0.0 for other machines)")
    command.add_argument("--web-port", type=int, help="port the browser viewer listens on")


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


def nubots_config(nubots_target, name, default=None):
    """A config file from the NUbots build, or `default` if it has none (an error without one)."""
    user = getpass.getuser()
    found = subprocess.run(
        ["docker", "run", "--rm", "--entrypoint", "cat",
         "--volume", f"nubots_{nubots_target}_{user}_build:{CONTAINER_BUILD}",
         f"{user}/nubots:{nubots_target}", f"{CONTAINER_BUILD}/config/{name}"],
        capture_output=True, text=True,
    )  # fmt: skip
    if found.returncode == 0:
        return found.stdout
    if default is None:
        sys.exit(f"error: the NUbots {nubots_target} build has no config/{name}. Build NUbots first (./b build there)")
    return default


def yolo_config(choice, nubots_target, session):
    """Where Yolo should run, and the Yolo.yaml to mount for it (None keeps NUbots' own, on the GPU).

    NUbots runs Yolo through TensorRT. Its image has TensorRT 10.3, which predates the RTX 50 series
    and fails on it ("Unsupported SM: 0xc00"), leaving the robots blind. Those GPUs fall back to the CPU.
    """
    if choice == "auto":
        try:
            gpu = subprocess.run(
                ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], capture_output=True, text=True
            ).stdout.strip()
        except FileNotFoundError:
            gpu = ""
        if not gpu:
            choice = "cpu"
            print("Vision: no NVIDIA GPU found, so Yolo runs on the CPU (--vision-device gpu overrides)", flush=True)
        elif re.search(r"RTX 50\d\d", gpu):
            choice = "cpu"
            print(
                f"Vision: {gpu} is too new for the TensorRT in NUbots' image, so Yolo runs on the CPU "
                "(--vision-device gpu overrides)",
                flush=True,
            )
        else:
            choice = "gpu"
    if choice == "gpu":
        return None
    stock = nubots_config(nubots_target, "Yolo.yaml")
    path = session / "Yolo.yaml"
    path.write_text(re.sub(r"^device:.*$", "device: cpu", stock, flags=re.M))
    return path


def config_value(file, key):
    """A top-level scalar from a config file, which needn't parse the rest of it."""
    return re.search(rf"^{key}:\s*([^\s#]+)", (MUJOCO / "config" / file).read_text(), re.M).group(1)


def field_value(key):
    return float(re.search(rf"^  {key}:\s*(\S+)", (MUJOCO / "config/match.yaml").read_text(), re.M).group(1))


def field_description(nubots_target):
    """NUbots' FieldDescription.yaml, changed to describe the sim's field."""
    stock = nubots_config(nubots_target, "FieldDescription.yaml")

    preset = "".join(f"    {name}: {field_value(key)}\n" for name, key in FIELD_PRESET.items())
    preset += f"    CenterCircleDiameter: {2 * field_value('center_circle_radius')}\n    GoalDepth: 0.6\n"
    text = re.sub(r"^field_type:.*$", "field_type: nusim", stock, flags=re.M)
    text = re.sub(r"^BallRadius:.*$", f"BallRadius: {field_value('ball_radius')}", text, flags=re.M)
    text = re.sub(r"^LineWidth:.*$", f"LineWidth: {field_value('line_width')}", text, flags=re.M)
    return text.replace("presets:\n", f"presets:\n  nusim:\n{preset}", 1)


def ground_truth_config(nubots_target):
    """The NUbots build's own NUSimGroundTruth.yaml, for start_robot to change per robot.

    Starting from the build's file keeps every key its module reads. A file written from scratch here goes stale
    whenever NUbots adds a key: the module's config then fails to load and it listens on no topic at all, so the
    robots quietly fall back to vision and localisation.
    """
    stock = nubots_config(nubots_target, "NUSimGroundTruth.yaml", default="")
    return stock if "ground_truth:" in stock else GROUND_TRUTH


def robot_communication(nubots_target):
    """NUbots' RobotCommunication.yaml, with team messages sent to the match network's broadcast address."""
    text = set_key(nubots_config(nubots_target, "RobotCommunication.yaml"), "broadcast_ip", f'"{broadcast_address()}"')
    return set_key(text, "log_level", os.environ.get("K1SIM_ROBOT_COMMS_LOG", "INFO"))


def set_key(text, key, value):
    """A config file's text with one top-level key set, added if the file doesn't have it."""
    line = f"{key}: {value}"
    if re.search(rf"^{key}:", text, flags=re.M):
        return re.sub(rf"^{key}:.*$", line, text, flags=re.M)
    return f"{text.rstrip()}\n{line}\n"


def subnet():
    return ipaddress.ip_network(SUBNET)


def broadcast_address():
    return str(subnet().broadcast_address)


def sim_address():
    return str(subnet()[2])


def robot_address(robot):
    return str(subnet()[10 + robot["robot_id"]])


def ensure_network():
    """The match network, created if it isn't there (or remade if its subnet changed)."""
    found = subprocess.run(
        ["docker", "network", "inspect", NETWORK, "--format", "{{range .IPAM.Config}}{{.Subnet}}{{end}}"],
        capture_output=True,
        text=True,
    )
    if found.returncode == 0 and found.stdout.strip() == SUBNET:
        return
    if found.returncode == 0:
        subprocess.run(["docker", "network", "rm", NETWORK], stdout=subprocess.DEVNULL, check=True)
    gateway = str(subnet()[1])
    subprocess.run(
        ["docker", "network", "create", "--driver", "bridge", "--subnet", SUBNET, "--gateway", gateway, NETWORK],
        stdout=subprocess.DEVNULL,
        check=True,
    )


def robot_container_flags(robot):
    """A robot's address on the match network, and its share of the CPU.

    It shares the sim's IPC namespace for the camera segments. When the CPU is short (3v3 takes all of a 12-thread
    one), the sim's physics goes first: at real time it is what every robot depends on. ROBOT_CPU_SHARES against the
    sim's default 1024.
    """
    return [
        "--network", NETWORK, "--ip", robot_address(robot), "--ipc", f"container:{SIM_CONTAINER}",
        "--cpu-shares", str(ROBOT_CPU_SHARES),
    ]  # fmt: skip


def wait_for_sim(timeout=60):
    """Wait for the sim's container, which robots join for IPC. False if it never started."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        running = subprocess.run(
            ["docker", "inspect", "--format", "{{.State.Running}}", SIM_CONTAINER], capture_output=True, text=True
        ).stdout.strip()
        if running == "true":
            return True
        time.sleep(0.5)
    return False


def roster(game, teams=(125, 126), players=(None, None)):
    """The roster: config::robot_roster()'s, with the given team numbers, and each team's player numbers (1 to game
    by default)."""
    first_domain = int(config_value("dds.yaml", "domain"))
    numbers = [list(team or range(1, game + 1)) for team in players]
    return [
        dict(robot_id=i + 1, team_id=teams[i // game], player_id=numbers[i // game][i % game], domain=first_domain + i)
        for i in range(2 * game)
    ]


def team_numbers(setup):
    """Each team's GameController number: its codebase's, or 125 and 126 for two teams of the same codebase."""
    home, away = TEAM_NUMBERS[setup["home_codebase"]], TEAM_NUMBERS[setup["away_codebase"]]
    return (home, away) if home != away else (125, 126)


def write_roster(path, robots):
    """A --match roster file for the sim. The first team attacks +x."""
    teams = sorted({robot["team_id"] for robot in robots}, key=[robot["team_id"] for robot in robots].index)
    lines = ["schema_version: 1", "teams:"]
    lines += [f"  - team_id: {team}\n    initial_attacks_positive_x: {'true' if i == 0 else 'false'}"
              for i, team in enumerate(teams)]  # fmt: skip
    lines += ["robots:"]
    lines += [f"  - {{robot_id: {r['robot_id']}, team_id: {r['team_id']}, player_id: {r['player_id']}, "
              f"dds_domain: {r['domain']}, state_every: {r.get('state_every', 1)}}}" for r in robots]  # fmt: skip
    path.write_text("\n".join(lines) + "\n")


def sim_path(path):
    """Where a file under mujoco/ is for the sim: in its container, the repo is at /workspace/NUSim."""
    return str(path) if backend() == "native" else f"/workspace/NUSim/mujoco/{path.relative_to(MUJOCO)}"


def image_available(image):
    """Whether a team's image is on this machine."""
    return subprocess.run(["docker", "image", "inspect", image], capture_output=True).returncode == 0


def start_pose(slot):
    """Where the sim spawns a team's robot in slot `slot` (config::game() in shared/util/Config.hpp), in the team's own
    field frame: "x y yaw", metres and radians. Each team lines up on the touchlines of its own half."""
    text = (MUJOCO / "config/simulation.yaml").read_text()
    game = re.search(r"^game:\n((?:[ #].*\n|\n)*)", text, re.M).group(1)

    def value(key):
        return float(re.search(rf"^  {key}:\s*([^\s#]+)", game, re.M).group(1))

    side = -1 if (slot // 2) % 2 == 0 else 1
    x = -value("length") / 2 + value("penalty_mark_distance") + ((slot // 4) * 2 + slot % 2) * value("sideline_spacing")
    y = side * (value("width") / 2 + value("sideline_offset"))
    return f"{x:.3f} {y:.3f} {-side * math.pi / 2:.4f}"


def sim_config(session):
    """A copy of config/ with LowState at 500 Hz, for a team whose motion runs at that rate."""
    directory = session / "config"
    shutil.rmtree(directory, ignore_errors=True)
    shutil.copytree(MUJOCO / "config", directory)
    simulation = directory / "simulation.yaml"
    simulation.write_text(set_key(simulation.read_text(), "state_publish_divisor", "2"))
    return directory


def gpu_flags():
    """GPU access for NUbots' vision, found the same way as docker/k1sim.sh's."""
    if list(pathlib.Path("/etc/cdi").glob("nvidia*.yaml")) + list(pathlib.Path("/var/run/cdi").glob("nvidia*.yaml")):
        return ["--device", "nvidia.com/gpu=all", "-e", "NVIDIA_DRIVER_CAPABILITIES=all"]
    runtimes = subprocess.run(["docker", "info", "--format", "{{json .Runtimes}}"], capture_output=True, text=True)
    return ["--gpus", "all", "-e", "NVIDIA_DRIVER_CAPABILITIES=all"] if '"nvidia"' in runtimes.stdout else []


def camera_segment(robot):
    """The robot's camera segment (MatchCameras::segment)."""
    base = config_value("camera.yaml", "segment")
    return base if robot["robot_id"] == 1 else f"{base}_robot{robot['robot_id']}"


def start_robot(codebase, robot, slot, role, session, nubots_target, ground_truth, first_team):
    """Start one robot's player in its own container, with its output in robot<id>/log."""
    directory = session / f"robot{robot['robot_id']}"
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "fastdds.xml").write_text(PROFILE.format(port_base=PORT_BASE + DOMAIN_GAIN * robot["domain"]))
    if codebase == "Nobody":
        return None
    if "install" in TEAMS[codebase]:
        command = team_command(codebase, robot, slot, directory)
    else:
        command = nubots_command(robot, role, directory, session, nubots_target, ground_truth, first_team)
    log = (directory / "log").open("w")
    return subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)


def team_command(name, robot, slot, directory):
    """Another team's player for one robot: its image, with the environment every team's gets
    (docs/MATCH_WINDOW.md#codebases), which its fork's nusim/run.sh starts it from."""
    gdb = bool(os.environ.get("K1SIM_TEAM_GDB"))
    return [
        "docker", "run", "--rm",
        "--name", container(robot),
        *robot_container_flags(robot),
        *TEAMS[name].get("docker", []),
        # As root, FastDDS shared memory with the sim fails.
        "--user", f"{os.getuid()}:{os.getgid()}",
        "--env", "FASTRTPS_DEFAULT_PROFILES_FILE=/robot/fastdds.xml",
        "--env", f"NUSIM_TEAM={robot['team_id']}",
        "--env", f"NUSIM_PLAYER={robot['player_id']}",
        "--env", f"NUSIM_ROBOT_ID={robot['robot_id']}",
        "--env", f"NUSIM_CAMERA={camera_segment(robot)}",
        "--env", f"NUSIM_START_POSE={start_pose(slot)}",
        *(["--env", "NUSIM_GDB=1", "--cap-add", "SYS_PTRACE"] if gdb else []),
        "--volume", f"{directory}:/robot:ro",
        team_image(name),
    ]  # fmt: skip


def nubots_command(robot, role, directory, session, nubots_target, ground_truth, first_team):
    """NUbots' role for one robot."""
    (directory / "NUSimCamera.yaml").write_text(CAMERA.format(segment=camera_segment(robot)))
    # NUbots' field frame has its own goal on +x. The first team defends the sim's -x goal.
    yaw = math.pi if first_team else 0.0
    truth = (session / "NUSimGroundTruth.yaml").read_text()
    for key, value in (
        ("ground_truth", "true"),
        ("world_to_field_yaw", yaw),
        ("log_level", os.environ.get("K1SIM_GROUND_TRUTH_LOG", "INFO")),
    ):
        truth = set_key(truth, key, value)
    (directory / "NUSimGroundTruth.yaml").write_text(truth)
    truth_config = f"{CONTAINER_BUILD}/config/NUSimGroundTruth.yaml"
    truth_mount = ["--volume", f"{directory}/NUSimGroundTruth.yaml:{truth_config}:ro"] if ground_truth else []

    user = getpass.getuser()
    return [
        "docker", "run", "--rm",
        "--name", container(robot),
        *robot_container_flags(robot),
        "--user", f"{os.getuid()}:{os.getgid()}",
        "--workdir", CONTAINER_BUILD,
        "--env", "FASTRTPS_DEFAULT_PROFILES_FILE=/robot/fastdds.xml",
        "--volume", f"nubots_{nubots_target}_{user}_build:{CONTAINER_BUILD}",
        "--volume", f"{directory}:/robot:ro",
        "--volume", f"{directory}/NUSimCamera.yaml:{CONTAINER_BUILD}/config/NUSimCamera.yaml:ro",
        "--volume", f"{session}/FieldDescription.yaml:{CONTAINER_BUILD}/config/FieldDescription.yaml:ro",
        "--volume", f"{session}/RobotCommunication.yaml:{CONTAINER_BUILD}/config/RobotCommunication.yaml:ro",
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


def container(robot):
    return f"nusim-match-robot{robot['robot_id']}"


def stop_containers(robots):
    names = [SIM_CONTAINER, *(container(robot) for robot in robots)]
    subprocess.run(["docker", "rm", "--force", *names], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def interrupt(signum, frame):
    raise KeyboardInterrupt


def run_once(setup, session, nubots_target, headless, rtf, web):
    """One sim session with its robots. Returns the window's next setup, or None once the sim just exits."""
    mode = setup["mode"]
    stop_containers(roster(3))  # from a run that didn't finish, whatever its size

    request = session / "request.json"
    request.unlink(missing_ok=True)
    (session / "session.json").write_text(json.dumps(setup, indent=2))

    run_flags = f"--name {SIM_CONTAINER}"
    game_role = next((role for role in NUBOTS_GAME_ROLES if role in setup["roles"]), "nusim/behaviour")
    codebases = (setup["home_codebase"], setup["away_codebase"])
    if mode == "game":
        # The automatic referee. Players that obey the GameController mustn't be held still between phases.
        role = "bin/sim/match"
        # Robots start on the touchlines and walk on when the game is Ready.
        sim_args = ["--game", str(setup["game"])]
        sim_args += ["--free-robots"] if game_role == "nusim/robocup" or set(codebases) - {"NUbots"} else []
        roles = (game_role, game_role)
    elif mode == "testing":
        # The built-in GameController, operated by hand from the window (or the browser viewer).
        role = "bin/sim/testing"
        # Robots start on the touchlines here too, and walk on when you call Ready.
        sim_args = ["--game", str(setup["game"]), "--referee"]
        roles = (setup["home_role"], setup["away_role"])
    else:
        # Just the sim: one robot on the chosen field, for a role started by hand from a terminal.
        role = "bin/sim/soccer"
        # --gamecontroller only listens: a GameController on the network is obeyed if there is one.
        sim_args = ["--field", setup["field"], "--gamecontroller"]
        sim_args += ["--robots", str(setup["robots"])] if setup["robots"] > 1 else []
        roles = None
    players = (setup.get("home_players"), setup.get("away_players"))
    robots = roster(setup["game"], team_numbers(setup), players) if roles else []

    # With a team whose motion runs at 500 Hz (B-Human) the sim publishes LowState at that rate. Every other robot still
    # gets the usual 50 Hz (every 10th): NUbots at 500 Hz loads its stack, and its mode requests start timing out.
    def fast(codebase):
        return TEAMS[codebase].get("low_state_hz") == 500

    fast_low_state = any(fast(codebase) for codebase in codebases)
    for robot in robots:
        codebase = codebases[0] if robot["team_id"] == robots[0]["team_id"] else codebases[1]
        robot["state_every"] = 10 if fast_low_state and not fast(codebase) else 1
    if robots:
        write_roster(session / "roster.yaml", robots)
        sim_args += ["--match", sim_path(session / "roster.yaml")]
    env = dict(os.environ, K1SIM_RUN_FLAGS=run_flags, K1SIM_DOCKER_TTY="-i")  # -i: a tty would eat ^C
    if robots:
        # The sim and its robots on the match network. Only the browser viewer's port is published, on this machine.
        ensure_network()
        # The GameController's control messages go to the match network only.
        sim_args += ["--broadcast", broadcast_address()]
        sim_args += ["--config-dir", sim_path(sim_config(session))] if fast_low_state else []
        network = f"--network {NETWORK} --ip {sim_address()} --ipc shareable"
        if web["enabled"]:
            network += f" --publish {web['host']}:{web['port']}:{web['port']}"
            sim_args += ["--web", "--web-host", "0.0.0.0", "--web-port", str(web["port"])]
        env["K1SIM_NETWORK_FLAGS"] = network
    elif web["enabled"]:
        sim_args += ["--web", "--web-host", web["host"], "--web-port", str(web["port"])]
    sim_args += ["--session", f"{build_dir()}/match"]
    sim_args += ["--headless"] if headless else []
    sim_args += ["--rtf", rtf] if rtf else []

    sim = subprocess.Popen([str(MUJOCO / "docker/k1sim.sh"), "run", role, *sim_args], env=env)

    try:
        time.sleep(SIM_STARTUP_SECONDS)
        if robots and not wait_for_sim():
            print("ERROR: the sim's container didn't start, so no robots were started", flush=True)
            robots = []
        processes = {}
        for index, robot in enumerate(robots):
            first_team = robot["team_id"] == robots[0]["team_id"]
            process = start_robot(
                codebases[0] if first_team else codebases[1],
                robot,
                index % setup["game"],  # its slot in its team's lineup
                roles[0] if first_team else roles[1],
                session,
                nubots_target,
                setup["ground_truth"],
                first_team,
            )
            if process is not None:
                processes[robot["robot_id"]] = process
        if robots:
            names = [codebase if codebase != "NUbots" else role for codebase, role in zip(codebases, roles)]
            print(
                f"{len(robots)} robots starting ({names[0]} v {names[1]}). Logs are in {session}/robot<id>/log",
                flush=True,
            )
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
    for key in (
        "game",
        "mode",
        "home_role",
        "away_role",
        "home_codebase",
        "away_codebase",
        "ground_truth",
        "field",
        "robots",
    ):
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


def parse_players(text, game):
    """--home-players / --away-players: game distinct player numbers, 1 to 11."""
    if not text:
        return None
    numbers = [int(number) for number in text.split(",")]
    if len(numbers) != game or len(set(numbers)) != game or not all(1 <= number <= 11 for number in numbers):
        sys.exit(f"error: player numbers need {game} distinct numbers from 1 to 11, got {text}")
    return numbers


def run(
    game,
    home_codebase,
    away_codebase,
    home_players,
    away_players,
    home_role,
    away_role,
    nubots_target,
    mode,
    ground_truth,
    vision,
    vision_device,
    replace,
    headless,
    rtf,
    web,
    web_host,
    web_port,
    **kwargs,
):
    # Only one match fits on a machine: the containers, DDS domains and camera segments are shared names.
    # Starting a second would silently kill the first one's sim and robots.
    running = subprocess.run(
        ["docker", "ps", "--quiet", "--filter", f"name=^{SIM_CONTAINER}$"], capture_output=True, text=True
    ).stdout.strip()
    if running and not replace:
        sys.exit("error: a ./b match is already running on this machine. Close its window first, or pass --replace")

    if not (MUJOCO / build_dir() / "bin/sim/match").exists():
        sys.exit("error: no bin/sim/match. Run ./b image, ./b configure --clean and ./b build")

    session = MUJOCO / build_dir() / "match"
    session.mkdir(parents=True, exist_ok=True)
    (session / "FieldDescription.yaml").write_text(field_description(nubots_target))
    (session / "NUSimGroundTruth.yaml").write_text(ground_truth_config(nubots_target))
    (session / "RobotCommunication.yaml").write_text(robot_communication(nubots_target))
    (session / "Yolo.yaml").unlink(missing_ok=True)
    yolo_config(vision_device, nubots_target, session)

    # Closing the terminal or killing this script mustn't leave containers behind.
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)

    web = dict(enabled=bool(web or web_host or web_port), host=web_host or "127.0.0.1", port=web_port or 8080)

    for codebase in CODEBASES:
        if "install" in TEAMS[codebase["name"]]:
            codebase["available"] = image_available(team_image(codebase["name"]))
            codebase["note"] = "" if codebase["available"] else codebase["note"]
    available = [codebase["name"] for codebase in CODEBASES if codebase["available"]]
    for codebase in (home_codebase, away_codebase):
        if codebase not in available:
            sys.exit(f"error: {codebase} can't be started here. Available: {', '.join(available)}")

    roles = available_roles(nubots_target)
    game_role = next((role for role in NUBOTS_GAME_ROLES if role in roles), "nusim/behaviour")
    setup = dict(
        game=game,
        mode=mode,
        home_role=home_role or game_role,
        away_role=away_role or game_role,
        ground_truth=ground_truth or (mode == "game" and not vision),
        home_codebase=home_codebase,
        away_codebase=away_codebase,
        home_players=parse_players(home_players, game),
        away_players=parse_players(away_players, game),
        field=FIELDS[0],
        robots=1,
        roles=roles,
        codebases=CODEBASES,
        fields=FIELDS,
    )
    try:
        while setup is not None:
            setup = run_once(setup, session, nubots_target, headless, rtf, web)
    except KeyboardInterrupt:
        pass
    finally:
        stop_containers(roster(3))
