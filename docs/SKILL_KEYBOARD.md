# Keyboard play with the mjlab policies

`./b skills` (the `sim/skills` role) puts one K1 on the field and lets you play it from the keyboard with the
walk, receive, dribble and kick policies from mjlab. No NUbots role runs and nothing goes over DDS: the policies
run inside the sim.

```bash
./b build
./b skills                 # same as ./b run sim/skills; takes the usual flags (--field, --rtf, ...)
./b skills sync            # copy the policies ~/mjlab's controller spec names (or --mjlab <dir>)
```

The policies and the rules for using them come from mjlab's `policies/CONTROLLER_INTERFACE.md` (the version of
2026-11-17 18:00).

## Keys

Click the sim window first so it has the keyboard.

| Key | What it does |
| --- | --- |
| `W` `A` `S` `D` | Without the ball (over a metre away, or none rolling in): walk. `W` forward at 0.6 m/s, `S` back at 0.3 m/s, `A`/`D` turn at 0.8 rad/s. Walking up to a ball picks it up, and with a direction still held it carries on into a dribble. |
| `X` | Release the ball: from receive, dribble or a shot being wound up, straight back to walking. The robot won't pick that ball up again until it has been 1.2 m away from it. |
| `Q` | Rolls the ball at the robot from wherever it is now (if it's more than a metre away), standing or walking. The receive policy traps it. A pass from behind the robot, from wider than 45 degrees or from more than 4 m is still played, with a warning: receive was trained only on passes from in front within 4 m. |
| `E` | Puts a ball at rest half a metre in front of the robot. |
| `W` `A` `S` `D` | With the ball at the robot's feet: dribble. The ball goes away from / to the left of / towards / to the right of the camera at 0.8 m/s. Release everything to hold the ball still. |
| `Shift` | Faster: walk forward at 1.0 m/s, dribble at 1.3 m/s. |
| `Space` | With the ball in front of the robot: hold to wind up a shot, release to kick. A tap is 2 m/s; the power climbs to 8 m/s over 2 s. An arrow from the ball shows the aim (it starts at the +x goal) and grows and reddens with the power; `W` `A` `S` `D` swing it while `Space` is held. The robot holds the ball until the release. |
| `R` | Stands the robot back up where it started. This also happens by itself 3 s after a fall. |
| `P` | Pause. (`Space` pauses in the other roles; here it shoots.) |

The current skill and what it is doing are shown at the top left of the window.

A ball that goes fully over a goal line between the posts and under the bar is a goal: "GOAL!" comes up in the middle
of the window, and confetti blasts out of both posts with a few fireworks above the goal. It's only for show, drawn by
the viewer and not part of the physics, so it can't move the ball or the robot. `Q` rolls the ball back out of the
net to the robot.

The spec trained the walk forward up to 1 m/s and found turning and sideways barely respond in mjlab. Measured in
NUSim:

| Asked | Got |
| --- | --- |
| Back, 0.3 m/s | 0.19 m/s back |
| Forward 0.6 m/s and turn 0.8 rad/s | 0.83 rad/s while walking |
| Turn 0.8 rad/s standing still | nothing; it turns only while stepping |
| Turn 0.8 rad/s plus 0.3 m/s forward | 0.70 rad/s, on a 0.26 m circle |
| Sideways 0.3 m/s | nothing |

So `A`/`D` on their own also ask for 0.3 m/s forward (`walk.turn_creep`), which turns the robot nearly on the spot.
`walk.side_keys: strafe` makes `A`/`D` step sideways instead, but the walk doesn't respond to that.

## Camera

The Simulation panel has three camera buttons. Keyboard play starts in **Behind**.

- **Free**: the camera stays wherever the mouse leaves it.
- **Follow**: the camera stays on the robot; the mouse still turns and zooms it.
- **Behind**: the camera stays behind the robot and turns with it. Here `W` means the robot's forward for walking
  and dribbling alike; in the other two views walking is along the robot's heading but dribbling follows the camera.

## How the skills are switched

The rules are in `module/SkillPlayer/src/SkillLogic.hpp`:

- The robot starts in **walk**, standing still.
- `X` goes from any ball skill back to walk.
- `Q` (a pass) or `E` (a ball at the feet) switches to **receive** at once. Walk to receive is a trained hand-over;
  the robot then stays in receive for 0.3 s whatever the keys, while the ball is moved.
- Walking with a ball within 0.8 m ahead also switches to receive.
- Receive hands over to **dribble** once the ball has sat within 18 cm of a point 18 cm in front of the robot, slower
  than 0.3 m/s, for half a second. A direction key takes over sooner if the ball is within a metre and already slower
  than 0.8 m/s.
- Holding `Space` with the ball within a metre and within 60 degrees of the robot's heading winds up a shot: the
  robot is asked to hold the ball still while the power and aim are set. Releasing it switches to **kick** (dribble
  to kick is trained). The aim stays within 135 degrees of the heading, or 45 degrees once the power is above 5 m/s.
  The wind-up is dropped if the ball rolls out of reach.
- After a kick the robot stands still. A direction key picks the dribble back up from a second after the shot if the
  ball is still within a metre; otherwise `W` walks.
- A direction key also leaves receive (with no pass rolling in) or dribble for walk if the ball is more than a metre
  away.

Switching to or from walk changes the PD gains, the default pose and action scale, and the input layout in the same
step, as the spec's section 4 describes. The new policy's "previous action" is the joint target last sent, in its
own convention, and a ball skill's three-step history restarts from the current state. Getting into walk from a ball
skill, and walking up to a resting ball, are not hand-overs mjlab has trained or tested.

## Settings

`mujoco/config/skills.yaml` holds the policy files, walk and dribble speeds, the frame the dribble keys work in
(`view`, `field` or `robot`), the shot's power range, charge time and aim, and the range of passes `Q` draws from.

`action_delay` (25 ms) holds each ball skill's joint targets back before they reach the servos. They were trained
with 10 to 40 ms there; sent at once, the dribble's leg targets moved 0.167 rad per control step on average, against
0.129 with the delay and 0.124 for the same policy and command in mjlab. The walk has its own `walk.action_delay`,
0 as in mjlab's walk demo.

It also takes a `script` of timed key presses, for running without a keyboard:

```yaml
script:
  - [1.5, press, W]
  - [5.0, tap, Q]
  - [5.4, release, W]
  - [9.0, press, space]
  - [10.5, release, space]
```

With `--headless` the module logs one `SKILL` line a second (`log_period`) giving the skill, the command, and where
the robot and ball are.

## Policies

`./b skills sync` reads the spec's policy table and copies each file it names into `mujoco/policies/skills/` as
`walk.onnx`, `receive.onnx`, `dribble.onnx` and `kick.onnx`, and records where each came from in `SOURCES.txt`.
For the kick it also checks where the spec says the finished run will be, and takes whichever file is newer, so
running sync again after mjlab's kick training ends picks up the final kick.

Joint order, default pose and action scale are read from each file's metadata. So are the PD gains, except that the
ball-skill files exported with the motor torque-speed model carry placeholder gains (kp 1, kd 0). For those the
module uses `ball_skill_gains` from the config, which are the spec's section 4 values, and says so in the log at
start-up. A file with a different joint order or input size stops the sim at start-up with the reason.

## What to expect

From scripted runs on the middle field with the policies of 2026-11-17 18:00:

- **Receive:** 12 of 12 passes and 6 of 6 balls put at the feet were trapped and handed over to dribble in one
  batch; one later pass, played while the robot was turning mid-dribble, was not.
- **Kick:** 18 of 18 shots struck the ball, from a dribble, from standing and straight after a pass, with no falls.
  Asked for 4.4 to 8 m/s, the ball left at 3.7 to 6.6 m/s. That is slower than asked, unlike mjlab's 1.2x
  overshoot; the spec's note that shots above about 5 m/s squash the ball into mjlab's softer ground is the likely
  difference.
- **Walk:** 0.6 m/s asked gave about 0.43 m/s.
- **Passes from where the ball lies:** trapped from 1.4 m, 2.3 m and 5.4 m in front, and eventually from 1.5 m
  directly behind; not from 2.7 m behind and to the side.

The ball observation is ground truth cut to the field of view the policies were trained with (90 by 60 degrees from
the front of the head); the head camera image is not used.

Headless there is no camera, so `view` keys act as if it looked along +y (the window's starting view).

Keys work in the sim window only, not in the browser viewer (`--web`).
