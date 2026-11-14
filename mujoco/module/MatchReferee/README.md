# MatchReferee

Runs a game inside the sim, so no GameController app is needed. The GameController and automatic referee
(`game_controller_api`) advance with sim time and see the ball and robots directly: goals are counted, the ball is
placed for restarts, and the game starts by itself, going straight to Playing at every kick-off. Control messages go
out over UDP exactly as they do from the GameController app. The window shows the score, state and clock, with buttons
to start, pause or restart the game, and a message box at the bottom left lists goals, restarts and penalties. Each
robot has a label like `Home 1`.

## Usage

Linux only. Build NUbots_K1 first (`./b target generic`, `./b build`), then, in NUSim:

```bash
mujoco/tools/install_game_controller_api.sh [path to GameController]   # needs cargo; default ../GameController
./b configure
./b build
./b match --game 3                                   # 1 to 3 a side; starts once every robot is connected
./b match --game 1 --away-role <role>                # a different NUbots role for each team
./b match --game 2 --headless                        # no window
./b match --mode testing                             # you call the set plays from the window
./b match --ground-truth                             # robots skip vision + localisation
```

The window's Setup panel (press Change) picks the mode, the robots a side, each team's codebase and perception; Apply
and Restart closes the sim and `./b match` starts it again with the new setup.

- `--mode game` (default): `sim/match`, the automatic referee. The window's INITIAL / READY / SET / PLAYING buttons step
  each kick-off by hand. Every robot runs NUbots' `nusim/robocup` role (or `nusim/behaviour` if that isn't built).
- `--mode testing`: `sim/testing`, with module::Referee's built-in GameController operated by hand from the window (or
  the browser viewer, which also has the stadium and Village Green venues). Nothing is refereed automatically. Each team
  picks its own NUbots role.
- `--mode freeplay`: `sim/soccer` alone, one robot on a field you choose, for a role you start from a terminal. It can
  obey a GameController running elsewhere.

`--ground-truth` gives each robot a `NUSimGroundTruth.yaml` that switches NUbots' `input::NUSimGroundTruth` on: it takes
the robot's pose and the ball from `rt/nusim/gt/robot` and `rt/nusim/gt/ball` in place of vision and localisation, and
the role must include that module. Codebases other than NUbots are listed in `tools/match.py`'s `CODEBASES`; each needs
a way to start one player per robot before it can be chosen.

`./b match` starts `sim/match` and a hidden NUbots container per robot, and stops them all when the sim exits. Robot
output is in `mujoco/build-docker/match/robot<id>/log`, and it warns if a robot stops. Roles that need a terminal, such
as the keyboard-driven `nusim/localisation`, can't run this way. Without the install step, `sim/match` is skipped and
everything else builds as normal. Settings are in `config/match.yaml`. Don't run the GameController app at the same
time: both use port 3939.

## Stock NUbots

`./b match` needs no NUbots_K1 changes. Each container gets files that point stock binaries at their own robot:

- A FastDDS profile whose `portBase` is that of the robot's domain. NUbots always joins domain 0 and the SDK ignores a
  `domainId` in the profile, but ports are all that separates domains on the wire.
- A `NUSimCamera.yaml` naming the robot's segment from `module::MatchCameras`.
- A `FieldDescription.yaml` for the sim's field, as NUbots defaults to its 6.8 x 5 m lab field.

Three things on the sim side support this:

- `hold_robots` in `match.yaml` stops robots where they are and moves them into the ready pose unless they're playing,
  because `nusim/behaviour` doesn't listen to GameController. Turn it off for a role that does. Robots aren't moved to
  kick-off positions.
- Under `--game`, a robot's odometry starts at zero in its start pose, as a real robot's does at boot. NUbots subtracts
  the first sample without rotating by its heading, which would otherwise reverse every robot that starts facing -x.
- New game re-requests each robot's mode, because NUbots requests it only once.

## Robot identity

The roster (`config::robot_roster()`) assigns teams and players: the first team is home, in the -x half. Team numbers
default to 125, which is also NUbots' default, and 126. Players are numbered in spawn order, so with
`--on-field-positions` player 1 is the attacker and player 2 the goalkeeper.

The referee only sees sim bodies, so it can't tell if a process has the wrong ID. A role that runs
`input::GameController` reports its ID, and the log shows any mismatch:

```
MatchReferee: online: team 125 player 1
MatchReferee: UNEXPECTED player online: team 9 player 3
```

## Networking

Control messages go to 255.255.255.255 on `lo`. NUClear's `UDP::Broadcast` accepts that address, every process on the
machine receives it, and a firewall such as ufw doesn't block it. To reach robots on other machines, leave
`network.interface` empty and allow the traffic through the firewall.

## Cameras

`module::MatchCameras` renders every robot's head camera at `cameras_fps` (`match.yaml`), on the GPU. It asks EGL for the
GPU directly: `module::Camera`'s default display goes through the display server when `DISPLAY` is set, which on a
Wayland desktop means rendering on the CPU. It warns if frames fall below half the rate asked for, as robots that can't
see properly just look like they're behaving strangely. 3v3 at 30 fps uses all of a 20-thread CPU, so lower
`cameras_fps` if `RTF` in the window drops below 1.

## Limitations

- Only the first half is played: nothing starts the second half.
- The only penalties are local game stuck, for the closest robot when the ball hasn't moved for 10 s, and leaving the
  field. The rest are off, because `nusim/behaviour` can't react to them. A penalised robot is put at the edge of its
  own half and frozen for `penalty_seconds`.
- `nusim/behaviour` chases the ball alone, with no team play: it has no `purpose::Soccer` or `RobotCommunication`.
- A new game teleports robots, which NUbots' localisation can't see until it next resets.
- Each robot's NUbots stack uses about 2.8 GB of RAM, so 3v3 needs about 20 GB free.
