# The match window

`./b match` opens NUSim with control panels in the window and starts the robot software for you: one NUbots
process per robot, each in its own container, each driving its own robot. You choose a mode, the teams and how the
robots perceive the world from the window; nothing has to be edited by hand.

It builds on three pieces of other people's work, which it needs:

- **Refereed matches with independent robots** (`sim/match`, `module::MatchReferee`, `tools/match.py`): every robot
  has its own controller, sensors, camera and DDS domain, and an automatic referee runs the game.
- **The built-in GameController** (`module::Referee`): the official GameController's rules, operated by hand.
- **Ground truth topics** (`rt/nusim/gt/robot`, `rt/nusim/gt/ball`): where a robot and the ball really are.

## Setting up

Linux only. Do this once.

1. Build [NUbots](https://github.com/NUbots/NUbots) `main`, which has the K1 code (NUbots_K1 is merged into it):

   ```bash
   cd ~/NUbots
   ./b target generic   # the docker image, <you>/nubots:generic, with the Booster SDK
   ./b configure
   ./b build            # bin/nusim/robocup is the role games run
   ```

   The window runs `nusim/robocup` for games, and any role you pick in the Testing Room. If `./b configure` fails
   with `The current CMakeCache.txt directory /home/booster/build/CMakeCache.txt is different than the directory
   /home/nubots/build`, the build volume is from NUbots' image before the merge. Delete it and configure again:
   `docker volume rm nubots_generic_$USER_build`.
2. Build NUSim: `./b configure`, then `./b build`. The GameController both referees use is built into the docker
   image, at the commit pinned in `mujoco/docker/Dockerfile`. With an image from before that, run
   `./b image`, then `./b configure --clean` and `./b build`.

## Running

```bash
./b match --vision-device cpu                # Game mode, 1 v 1, ground truth (see "Load" for the flag)
./b match --game 2 --vision-device cpu       # two a side (up to three)
./b match --mode testing                     # start in the Testing Room
./b match --mode freeplay                    # start in Free Play
./b match --vision                           # Game mode with vision + localisation instead of ground truth
```

Wait for the robots before pressing **Start Match**: the terminal prints `MatchReferee: online: team 125 player 1`
and so on once each robot's NUbots has started and is listening to the GameController. On a game started before
that, the robots don't move.

Closing the window, or Ctrl+C in the terminal, stops the sim and every robot. Each robot's output is in
`mujoco/build-docker/match/robot<id>/log`. Only one match fits on a machine; a second `./b match` refuses to start
unless you pass `--replace`.

Press **Change** at the top left to open the setup: mode, robots a side, each team's codebase, perception and
field. **Apply and Restart** closes the sim and starts it again, with every robot's software, on the new setup.

## The three modes

### Game

A refereed match. Robots start on the touchlines and wait. **Start Match** is the only button you need:

1. **Ready**: the robots walk on to their kick-off positions.
2. **Set**: called once every robot has stood still for a second (or after 45 s). The ball goes to the centre mark.
3. **Playing**: two seconds later. The clock starts.

After that the automatic referee runs the game. It counts goals, calls throw-ins, corners and goal kicks, and
penalises robots; the Referee box says why each time. Every kick-off after a goal goes through the same Ready, Set,
Playing steps. A robot that lies on the ground for 10 seconds is stood back on the touchline and walks on again.
With one robot a side nobody is goalkeeper; with more, each team's second robot is.

You can't move robots or the ball by hand in this mode. Ground truth is on by default.

### Testing Room

You are the referee. The buttons are decisions for the built-in GameController, and each is enabled only when the
rules allow it; nothing is called automatically.

- **Kick-off**: READY, SET, PLAYING, for the team the rules give the kick-off to.
- **Set plays** (left panel): pick corner kick, goal kick, free kick, throw-in or penalty kick for a team. Play stops.
  Click a highlighted spot on the field to put the ball there. **Resume Play** sends the robots to their positions
  and lets that team take the kick. The ball is free for everyone once it has been kicked, or when you press
  **Ball Free**.
- **Penalise / Release a Robot**: press it, then click a robot. It goes to the touchline; do the same again to let it
  back on. Space pauses the sim, which makes a moving robot easier to click.
- Stop Play, Resume Play, goals, Finish Half, Switch Half, Game Stuck, timeouts and Undo.

Each team picks its own NUbots role. You can move robots and the ball by hand: double-click one for arrows and a
ring, or hold Ctrl and drag it. The browser viewer is served too, at `http://127.0.0.1:8080`, with its stadium and
Village Green venues.

### Free Play

Just the simulator: no referee and no robot software started for you. Choose the field and how many robots, then
run a role yourself from a terminal (`./b run <role>` in NUbots). A GameController running on the network is obeyed
if there is one.

Free Play needs the one-time setup above too, because the Setup panel can switch to the other modes. If it hasn't
been done, `./b match` says what is missing and prints the commands to run.

## Perception: vision or ground truth

- **Vision**: the robots find themselves and the ball with their own camera.
- **Ground truth**: the launcher switches NUbots' `input::NUSimGroundTruth` on for every robot. It takes the robot's
  pose and the ball from NUSim's ground truth topics and stands in for vision and localisation, so behaviour can be
  tested on its own. The role has to include that module.

Vision runs through TensorRT in NUbots' image. That TensorRT is too old for RTX 50-series cards, so on those the
launcher runs the vision model on the CPU instead (`--vision-device gpu` overrides this).

## Load

Every robot runs a full NUbots stack, including the Yolo vision model on every camera frame, which matters even with
ground truth (`nusim/robocup` runs Yolo either way).

- **GPU**: on a robot's first run, TensorRT builds an engine for Yolo, and every robot builds its own at once. On a
  12 GB RTX 3060 four of them took all of its memory and froze the desktop. NUbots caches the engine in `trt_cache/`
  in its build volume, but nothing stops the robots building it together, and its TensorRT options set no workspace
  limit. Until that is fixed, run Yolo on the CPU with `--vision-device cpu`.
- **CPU**: with Yolo on the CPU each robot takes about 2.5 cores. A 2 v 2 on 12 threads runs, slowly; 3 v 3 needs
  more. `cameras_fps` in `mujoco/config/match.yaml` (10) sets how often Yolo runs. If `RTF` in the window drops well
  below 1, use fewer robots.
- **Memory**: about 5 GB of RAM per robot.

## Team communication

NUbots' robots tell their teammates where they are and whether they are going for the ball
(`network::RobotCommunication`): UDP on port `10000 + team number`, so 10125 and 10126 here, and each team hears
only its own. Its config broadcasts to the robot network's address, which this machine doesn't have, so the launcher
gives every robot a `RobotCommunication.yaml` that sends to `255.255.255.255`, which reaches every robot on the
machine. It can't be `127.0.0.1`, which reaches only one of the robots sharing a port, or `127.255.255.255`, which
NUClear's broadcast receiver drops because `lo` has no broadcast flag.

To see the messages, run with `K1SIM_ROBOT_COMMS_LOG=DEBUG`; each robot's log then shows
`Message received from teammate ID 2 position: ...`.

## Codebases

The team pickers list NUbots, rUNSWift and B-Human. Only NUbots can be started so far. To add another, give it an entry in
`CODEBASES` in `mujoco/tools/match.py` and a way to start one player per robot in `start_robot`.

## What NUbots needs

All of these are in NUbots `main`.

- `input::NUSimGroundTruth`, in the `nusim` roles.
- `roles/nusim/robocup.role`: `robocup.role` with the camera, walk, kick and get-up modules swapped for the
  simulator's. Everything that decides what to do is unchanged, including `input::GameController` and
  `network::RobotCommunication`.
- Two settings in `Soccer.yaml`, both off for the real robot and on for `nusim/robocup`
  (`module/purpose/Soccer/data/config/bin/nusim/robocup/Soccer.yaml`):
  - `firmware_soccer_mode: false`: the simulator has no firmware soccer mode, and a robot put in it stands still.
  - `fall_recovery_from_startup: true`: nobody presses the simulated robot's button, which is what otherwise starts
    fall recovery.

## Debugging a robot

`K1SIM_ROBOT_MOUNTS` swaps config files into every robot, for example a module's yaml with `log_level: DEBUG`:

```bash
K1SIM_ROBOT_MOUNTS="/path/to/FieldPlayer.yaml:config/FieldPlayer.yaml" ./b match --game 2
```

`K1SIM_GROUND_TRUTH_LOG=DEBUG` makes the ground truth module print where each robot believes it and the ball are.
`K1SIM_ROBOT_COMMS_LOG=DEBUG` does the same for team messages, sent and received.
