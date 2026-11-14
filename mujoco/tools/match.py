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
    command.add_argument("--home-role", default="nusim/behaviour", help="NUbots role for the first team")
    command.add_argument("--away-role", default="nusim/behaviour", help="NUbots role for the second team")
    command.add_argument("--nubots-target", default="generic", help="the NUbots_K1 ./b target that was built")
    command.add_argument("--headless", action="store_true", help="no window; the game starts by itself")
    command.add_argument("--rtf", help="real-time factor")


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


def start_robot(robot, role, session, nubots_target):
    directory = session / f"robot{robot['robot_id']}"
    directory.mkdir(parents=True, exist_ok=True)

    base = config_value("camera.yaml", "segment")
    segment = base if robot["robot_id"] == 1 else f"{base}_robot{robot['robot_id']}"
    (directory / "fastdds.xml").write_text(PROFILE.format(port_base=PORT_BASE + DOMAIN_GAIN * robot["domain"]))
    (directory / "NUSimCamera.yaml").write_text(CAMERA.format(segment=segment))

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


def run(game, home_role, away_role, nubots_target, headless, rtf, **kwargs):
    if not (MUJOCO / build_dir() / "bin/sim/match").exists():
        sys.exit("error: no bin/sim/match. Run tools/install_game_controller_api.sh, ./b configure and ./b build")

    robots = roster(game)
    session = MUJOCO / build_dir() / "match"
    session.mkdir(parents=True, exist_ok=True)
    (session / "FieldDescription.yaml").write_text(field_description(nubots_target))
    stop_containers(robots)  # from a run that didn't finish

    sim_args = ["--game", str(game), "--on-field-positions"]
    sim_args += ["--headless"] if headless else []
    sim_args += ["--rtf", rtf] if rtf else []
    # Closing the terminal or killing this script mustn't leave containers behind.
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)

    env = dict(os.environ, K1SIM_RUN_FLAGS=f"--name {SIM_CONTAINER}")
    sim = subprocess.Popen([str(MUJOCO / "docker/k1sim.sh"), "run", "bin/sim/match", *sim_args], env=env)

    try:
        time.sleep(SIM_STARTUP_SECONDS)
        processes = {}
        for robot in robots:
            role = home_role if robot["team_id"] == robots[0]["team_id"] else away_role
            processes[robot["robot_id"]] = start_robot(robot, role, session, nubots_target)
        print(f"{len(robots)} robots starting. Logs are in {session}/robot<id>/log", flush=True)

        # Their terminals are hidden, so say when one stops. Roles that need a keyboard can't run here.
        while sim.poll() is None:
            for robot_id, process in list(processes.items()):
                if process.poll() is not None:
                    print(f"WARNING: robot {robot_id} stopped, see {session}/robot{robot_id}/log", flush=True)
                    del processes[robot_id]
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_containers(robots)
