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

1. Build NUbots (`./b target generic`, `./b build` in your NUbots checkout). The window runs these roles from it:
   `nusim/robocup` for games, and any role you pick in the Testing Room.
2. Stage the GameController library, which both referees use. It needs Rust (`cargo`):

   ```bash
   git clone https://github.com/RoboCup-HumanoidSoccerLeague/GameController mujoco/.deps/GameController
   git -C mujoco/.deps/GameController checkout 21c7c6a868fe29cfdc345404859a5344b1857274
   mujoco/tools/install_game_controller_api.sh mujoco/.deps/GameController
   (cd mujoco/.deps/GameController && cargo build -r -p game_controller_api)
   ```

   The first two lines get the commit the referees were tested with. The script stages the library for the automatic
   referee; the `cargo build` leaves the copy the hand-operated one loads.
3. Build NUSim: `./b configure`, then `./b build`.

## Running

```bash
./b match                       # Game mode, 1 v 1, ground truth
./b match --game 2              # two a side (up to three)
./b match --mode testing        # start in the Testing Room
./b match --mode freeplay       # start in Free Play
./b match --vision              # Game mode with vision + localisation instead of ground truth
```

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

## Perception: vision or ground truth

- **Vision**: the robots find themselves and the ball with their own camera.
- **Ground truth**: the launcher switches NUbots' `input::NUSimGroundTruth` on for every robot. It takes the robot's
  pose and the ball from NUSim's ground truth topics and stands in for vision and localisation, so behaviour can be
  tested on its own. The role has to include that module.

Vision runs through TensorRT in NUbots' image. That TensorRT is too old for RTX 50-series cards, so on those the
launcher runs the vision model on the CPU instead (`--vision-device gpu` overrides this).

## Codebases

The team pickers list NUbots and rUNSWift. Only NUbots can be started so far. To add another, give it an entry in
`CODEBASES` in `mujoco/tools/match.py` and a way to start one player per robot in `start_robot`.

## What NUbots needs

- `input::NUSimGroundTruth`, in the `nusim` roles.
- `roles/nusim/robocup.role`: `robocup.role` with the camera, walk, kick and get-up modules swapped for the
  simulator's. Everything that decides what to do is unchanged.
- Two settings in `Soccer.yaml`, both off for the real robot and on for `nusim/robocup`:
  - `firmware_soccer_mode: false`: the simulator has no firmware soccer mode, and a robot put in it stands still.
  - `fall_recovery_from_startup: true`: nobody presses the simulated robot's button, which is what otherwise starts
    fall recovery.

## Debugging a robot

`K1SIM_ROBOT_MOUNTS` swaps config files into every robot, for example a module's yaml with `log_level: DEBUG`:

```bash
K1SIM_ROBOT_MOUNTS="/path/to/FieldPlayer.yaml:config/FieldPlayer.yaml" ./b match --game 2
```

`K1SIM_GROUND_TRUTH_LOG=DEBUG` makes the ground truth module print where each robot believes it and the ball are.
