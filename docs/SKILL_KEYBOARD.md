# Keyboard play with the mjlab policies

`./b skills` (the `sim/skills` role) puts one K1 on the field and lets you play it from the keyboard with the
policies from mjlab: walk, receive, directional first touch, first-time pass, dribble, kick and air shot. No NUbots
role runs and nothing goes over DDS: the policies run inside the sim.

```bash
./b build
./b skills                 # same as ./b run sim/skills; takes the usual flags (--field, --rtf, ...)
./b skills sync            # copy the policies ~/mjlab's controller spec names (or --mjlab <dir>)
```

The policies and the rules for using them come from mjlab's `policies/CONTROLLER_INTERFACE.md` (the version of
2026-11-18 17:45).

## Keys

Click the sim window first so it has the keyboard. The window starts with the camera behind the robot, where `W`
means forward for walking and dribbling alike.

| Key | What it does |
| --- | --- |
| `W` `A` `S` `D` | Without the ball (over a metre away, or none rolling in): walk. `W` forward at 1 m/s, `S` back at 0.3 m/s, `A`/`D` turn at 0.8 rad/s. Walking up to a ball picks it up. With the ball: dribble away from / left of / towards / right of the camera at 0.8 m/s; release everything to hold it still. |
| `Q` | Call for a pass: the ball is rolled at the robot from wherever it is. See [Passes](#passes) for what you can do as it comes. |
| `Space` | With a pass rolling in: shoot it first time (see [Passes](#passes)). With the ball in front of the robot: hold to wind up a shot, release to kick. A tap is 2 m/s, rising to 7 m/s over 2 s. An arrow from the ball shows the aim (it starts at the +x goal) and grows and reddens with the power; `W` `A` `S` `D` swing it while the key is held. |
| `C` | The same for an air shot: the ball lifts at about 30 degrees. |
| `X` | Let go of the ball and walk. |
| `E` | Puts a ball at rest half a metre in front of the robot. |
| `Shift` | Faster: walk at 2 m/s (past the 1 m/s its author reports; it tracks 1.9 m/s here without falling), dribble at 1.8 m/s, first touch at 1.2 m/s, first-time pass at 4 m/s. |
| `R` | Stands the robot back up where it started. This also happens by itself 3 s after a fall. |
| `P` | Pause. (`Space` pauses in the other roles; here it shoots.) |

## Passes

`Q` rolls the ball at the robot from where it lies, to arrive at 1.5 to 2.5 m/s. Receive was trained on passes from
in front, up to 4 m away and within about 48 degrees; a pass from behind, from wide or from further is still played,
with a warning. As it comes:

- **Do nothing:** the robot receives it, trapping it at its feet, and the dribble takes over once it is still.
- **Hold a direction** (any time before the ball reaches the robot): a **directional first touch** that sends the
  ball that way at 0.9 m/s (`Shift`: 1.2), then the dribble takes over 0.4 s after the touch. Holding the direction
  early is what the policy was trained on: it gets the command from the moment the pass is played. Directions are
  kept within 90 degrees of where the pass came from, sideways or back towards the passer, never straight on past
  the robot. An arrow from the robot shows the loaded direction; releasing the key keeps it.
- **Press `Q` again:** a **first-time pass** at 3 m/s (`Shift`: 4), aimed with the direction keys or, with none held,
  straight back to where it came from. The robot stands still afterwards, as after a kick, and a direction key gets
  it moving again a second later. Press it early: the policy was trained running from the moment the pass is played,
  and switching to it late, from receive, is a hand-over it has never seen (see the results below).
- **Press `Space`** (or `C`): a **first-time shot**. There is no first-time shooting policy, so this is the first-time
  pass policy at its top trained speed, 4 m/s, aimed at the +x goal, or the way you're holding. Like the pass, it
  stays within 90 degrees of where the pass came from, and it works best pressed early. There is no first-time air
  shot, so `C` does the same.

The status line at the top left says what is loaded.

## Training drills

When keyboard play is running, the Simulation panel has a **Training** section with three buttons. Pressing the
one already running starts it again.

- **Free play** is everything above: one robot, the ball where the scene puts it. The other robots stand on the -y
  touchline out of the way (`park` in the config).
- **Rondo**: three teammates stand at the corners of a 4 m triangle facing in, one with the ball at its feet, and
  the fourth stands in the middle. Pass it round and keep it away from the robot in the middle.
- **Defend**: the same rondo the other way round: you play the robot in the middle, the computer plays the three
  (see [Defending](#defending)).

The rondo needs four robots, so `./b skills` now spawns four unless you pass `--robots` (`--robots 1` gives the
single-robot scene, and the Rondo button then says why it can't start).

In the rondo:

- **The keys play whoever the ball is going to.** When a pass or first-time pass leaves the robot you're playing,
  its roll is predicted (rolling at 0.9 m/s²), and the teammate it will pass within 0.6 m of (`rondo.receive_reach`)
  before it stops receives it; the keys switch to that teammate. A pass that will reach nobody puts nobody in
  receive: the keys go, in walk, to the teammate nearest where it will stop, with "off target: walk to the ball". A cyan arrow hangs over the
  robot you're playing. Direction keys still held from the pass don't count for the new robot until you let them
  go, so a key held to aim the pass doesn't turn into a first touch.
- **Passes auto-aim at a teammate.** `Space` aims at whichever of the other two has the passing lane further from
  the defender, and a tap is powered to reach them at 1.8 m/s (`rondo.arrival_speed`). Hold for more power. `W A S D`
  still swing the aim, and when you let go within 10 degrees of a teammate it locks onto them. With a pass coming in,
  `Q` passes it first time to the open teammate, or, with a direction held, to the teammate within 30 degrees of
  that direction (else exactly that way). `Space` with a pass coming hits it first time the same way, at 4 m/s.
  Hold a direction with no `Q` for a first touch, as in free play.
- **The defender** walks at the ball with the walk policy, as fast as it goes (2 m/s), aiming where the ball will be
  when it gets there. It waits 2 s after a round starts, eases off near the ball so it doesn't run through the robot
  holding it, and changes speed at no more than 3 m/s². It doesn't kick: touching the ball wins it.
- **A round ends** when the defender's body touches the ball, or the ball rolls 2 m past the triangle's corners.
  Bumping into a robot doesn't count: a contact between the ball and the defender does, and so does a jump in the
  ball's speed between two control steps if the defender is then within 10 cm of it and nearer it than any teammate
  (`mj_geomDistance`). The next round starts 1.5 s later with the next teammate on the ball. `R` starts a new round at
  once. The status line counts the passes completed this round in a row (a pass counts when the teammate it was
  played to touches it), the best round, interceptions and lost balls.
- **The camera** switches to Free, looking over the triangle. Follow and Behind follow the robot you're playing.
  Switching back to free play puts the camera behind the robot again.
- A robot that falls is stood back up at its spot after 3 s.

A robot waiting on a pass that rolls past it, away from it, or stops out of its reach goes straight back to walk, in
free play too (`release_missed` in `SkillLogic.hpp`'s config: further than 0.7 m, coming no closer than 0.15 m/s,
0.4 s after the pass is played), so you can walk to the ball instead of standing in receive until it times out.

Teammates you're not playing stay available. After a pass they go back to walk and head for their corner of the
triangle, turned to face the ball so the next pass comes from in front, as receive was trained. Far from their corner
(over a metre) they turn and walk there at 0.8 m/s; closer in they step there sideways or backwards with the ball in
view, and they don't move again until they are 0.4 m off it. If the defender stands within 0.8 m of the lane from
the ball to a teammate's corner, that teammate steps sideways off the corner, away from the defender, until the lane
is clear (by up to 1.2 m), and goes back to the corner once it is. A pass played to a teammate on the move is the
trained walk-to-receive hand-over. `rondo:` in the config sets the triangle's size and centre, the
defender's speed, turn rate, head start and lead, the support movement (`support_speed`, `support_tolerance`,
`lane_clear`, `support_shift`) and the auto-aim. `drill: rondo` starts in the rondo.

## Defending

The **Defend** button (`drill: rondo_defend`) puts you in the middle. `W A S D` walk the robot there (`Shift` at
2 m/s), always with the walk policy: it never takes the ball, touching it wins it ("you won it after 3 passes in
6.2 s!"). So does the ball getting away before they have made 8 passes (`rondo.forced_win_passes`, here and in the
tracking rondos below): you pressed them into losing it. The cyan arrow hangs over it, and Follow and Behind follow it.

The computer plays the three teammates with the same policies and switching rules, through the same keys a person
would press:

- With the ball, it holds it for 0.6 to 1.8 s (`rondo.ai_hold`), then passes to the open teammate, with a tap of the
  pass key, if the defender is at least 0.7 m from that lane (`ai_lane_open`) and the ball sits still in front. With
  the defender within 1.6 m of the ball (`ai_pressure`) it passes at once if a lane is open; otherwise, within 2 m, it
  dribbles away from the defender, bending back towards its corner.
- A pass coming in with the defender within 1.8 m of the receiver (`ai_first_time`) is passed on first time.
- A kick that went nowhere, or a trap that never settled, is picked up with the dribble; a ball that got away is
  walked to at up to 0.8 m/s (`chase_speed`) and picked up.

Headless, with nobody defending, four 120 s runs in the web port made 16 to 23 passes each, with a pass reaching
nobody about once in five; rounds mostly ended with a poorly trapped pass rolling out.

## Defending against EKF tracking

The **Shared eyes (EKF)** button (`drill: rondo_ekf`) is the same defending rondo on a wider triangle (5.5 m,
`rondo_ekf.side`), except the three passers don't know where you are. All they have is what their head cameras see
of you, fused into one shared extended Kalman filter:

- **Detections.** Each teammate's head camera (the model's `head` camera, the K1's lens) sees you when your trunk is
  in its image, within 8 m, and a ray from the camera to your trunk or head isn't blocked by another robot or the
  ball. Even in plain view it misses 1 frame in 10. Each camera reports 15 times a second: a bearing (1.7 degrees of
  noise) and a range from your size in the image (0.1 m + 0.12 m per metre away). Each teammate knows where its own
  camera is.
- **The shared EKF** (`module/SkillPlayer/src/DefenderEkf.hpp`) tracks your position and velocity on the ground with
  a constant-velocity model (white acceleration noise, the velocity fading over 1.5 s with nobody seeing you). Every
  detection, from any of the three, is a range-bearing update from that camera's spot; one that is too far off the
  track (Mahalanobis gate) is thrown away, and after 8 in a row the track starts again from the latest.
- **The computer plays against the track,** not against you: the pressure, first-time and open-lane checks and the
  support movement all use the track, and lanes must be clear of it by its 1-sigma doubt as well. With a track vaguer
  than 2 m (`lost_sigma`) they assume you're in the middle. So standing behind one of them, or where none of them is
  looking, gets passes played into you.

What you see: the three cameras' views down the left of the window (the robot's number and whether it sees you,
with a box round you: green seen, yellow in view but hidden), and in the scene the track as an orange 2-sigma ellipse
with a see-through orange robot at its middle and an arrow for its velocity, each camera's last detection as a dot
in that camera's colour with a line back to it, and a red line from where you really are to the track. The status
line counts the cameras that see you and how far off the track is. All of it is in `rondo_ekf:` in the config.

**Solo eyes** (`drill: rondo_solo`) is the same with no communication: each teammate keeps its own EKF of only what
its own camera sees (drawn in that camera's colour) and makes its own decisions (passing, open lanes, where to
stand) from that alone. Nothing the three do uses the ground truth; only the referee does (the ball touched, the ball
gone). The browser build (k1-skills-web) has both, as Shared eyes and Solo eyes.

## Camera

The Simulation panel has three camera buttons:

- **Free**: the camera stays wherever the mouse leaves it.
- **Follow**: the camera stays on the robot (in the rondo, the one you're playing); the mouse still turns and zooms it.
- **Behind**: the camera stays behind that robot and turns with it.

## Goals

A ball that goes fully over a goal line between the posts and under the bar is a goal: "GOAL!" comes up in the middle
of the window, and confetti blasts out of both posts with a few fireworks above the goal. It's only for show.

## How the skills are switched

The rules are in `module/SkillPlayer/src/SkillLogic.hpp`:

- The robot starts in **walk**, standing still.
- `Q` or `E` switches to **receive**; with a direction held as the pass is played, straight to **first touch**
  instead (walk to receive and walk to first touch are trained hand-overs).
- A first touch or first-time pass is "touched" when the ball is in contact with the robot, or its velocity jumps
  near the robot between two control steps. No touch within 4 s: back to receive.
- Receive hands over to **dribble** once the ball has sat within 18 cm of a point 18 cm in front of the robot, slower
  than 0.3 m/s, for half a second. With no pass rolling in, a direction key takes over sooner if the ball is within
  a metre and slower than 0.8 m/s.
- Holding `Space` or `C` with the ball within a metre and within 60 degrees of the robot's heading winds up a shot;
  releasing it switches to **kick** or **air shot** (dribble to kick is trained). The aim stays within 135 degrees of
  the heading, or 45 degrees once the power is above 4.5 m/s.
- After a kick or first-time pass the robot stands still. A direction key picks the dribble back up a second later if
  the ball is within a metre; otherwise it walks.
- A direction key also leaves receive (with no pass rolling in) or dribble for walk if the ball is more than a metre
  away, and `X` goes from any ball skill to walk.

Switching to or from walk changes the PD gains, the default pose and action scale, and the input layout in the same
step, as the spec's section 4 describes. Untested in mjlab: getting into walk from a ball skill, walking up to a
resting ball, and switching from receive to first-time pass part-way through a pass.

## The ball's contact with the robot

The policies were trained (in mjlab, from 2026-11-18) with explicit foot-ball contacts of friction 0.3 and solref
(0.02, 0.35): a little springy, so the ball comes off a foot about 1.4x as fast as the foot hits it, like a real
kick. NUSim's scene has no foot-ball contact of its own, so there the ball has priority and its critically damped
contact applies: it barely springs off a foot at all. With that, first touches went in random directions and kicks
left at 0.7-0.8x the speed asked for.

So in this role only, `ball_contact.match_mjlab` (on by default) sets the ball's own friction and solref to mjlab's
foot-ball values when the model loads. The ball has priority in its contacts with the robot, so these decide them;
ball-floor contacts come from the scene's explicit `<pair>` and are unchanged (a free-rolling pass slows exactly as
before). No other role is affected.

## Settings

`mujoco/config/skills.yaml` holds the policy files, walk, dribble, first-touch and first-pass speeds, the frame the
dribble keys work in (`view`, `field` or `robot`), the shot's power range, charge time and aim, the passes `Q` plays
(`pass.from: spawn` rolls them from a fresh spot in front instead of from the ball), the ball contact and the camera
model (the K1's calibrated 106.2 x 95 degree field of view).

`action_delay` (25 ms) holds each ball skill's joint targets back before they reach the servos. They were trained
with 10 to 40 ms there; sent at once, the dribble's leg targets moved 0.167 rad per control step on average, against
0.129 with the delay and 0.124 for the same policy and command in mjlab. The walk has its own `walk.action_delay`,
0 as in mjlab's walk demo.

It also takes a `script` of timed key presses, for running without a keyboard:

```yaml
script:
  - [2.0, tap, Q]
  - [2.1, tap, Q] # a first-time pass
```

With `--headless` the module logs one `SKILL` line a second (`log_period`) giving the skill, the command, and where
the robot and ball are.

## Policies

`./b skills sync` copies mjlab's current best set into `mujoco/policies/skills/` as `walk`, `receive`, `dribble`,
`kick`, `kick_air`, `first_touch` and `first_pass` `.onnx`, and records where each came from in `SOURCES.txt`. The
ball skills are the checkpoints the first `| Skill | File |` table of mjlab's `policies/BENCHMARKS.md` names (a
`folder/model_<n>` is its `model_<n>.onnx`, or the folder's one export when that is the checkpoint's); a skill that
table doesn't name (the walk) comes from the spec's policy table. Leave `kick_air`, `first_touch` or `first_pass` empty in the config and that action is off.

Joint order, default pose and action scale are read from each file's metadata. So are the PD gains, except that the
ball-skill files exported with the motor torque-speed model carry placeholder gains (kp 1, kd 0). For those the
module uses `ball_skill_gains` from the config, and says so in the log at start-up. The 2026-11-20 set was trained
on the real robot's gains (legs 80 / 4, ankles 50 / 2, arms 10 / 1, head 4 / 0.25, the same as the walk's), which is
what the config now has; the 2026-11-19 set needed the spec's section 4 values instead. A file with a different joint order or input size stops the sim at start-up with the reason.

## What to expect

Scripted runs on the middle field with the policies of 2026-11-18 17:45 and the matched ball contact, 8 of each
unless noted; passes from 2.5 to 3.5 m in front.

| | Result |
| --- | --- |
| Receive | 8 of 8 trapped and handed over to dribble |
| First touch, standing | 8 of 8 touched; the ball within 15 degrees of the direction asked in 6, 29 and 67 degrees off in the other two; 0.6 to 1.2 m/s |
| First touch, walking | 8 of 8 touched, no falls; 5 to 37 degrees off (median about 20) |
| First-time pass, `Q` again at once | 6 of 8 within 2 degrees at 2.0 to 2.6 m/s (asked 3); 1 weak; 1 fall |
| First-time pass, `Q` again 0.4 s later | 5 of 8 within 15 degrees; 3 falls |
| First-time shot, `Space` at once, from 7 m out | 5 of 8 scored (within 2 degrees, 2.9 to 3.7 m/s); 3 falls, each with the ball arriving at the robot's feet. At 3.5 m/s instead of 4: 2 of 8 scored, 2 falls |
| Air shot | 4 of 4 within 3 degrees; asked 4.5 m/s, left at 5.6; rose to about 0.4 m |

Before the contact was matched, first touches mostly went the wrong way and the first-time pass managed 2 of 8.

The ball observation is ground truth cut to the camera's field of view; the head camera image is not used. Headless
there is no camera, so `view` keys act as if it looked along +y.

Keys work in the sim window only, not in the browser viewer (`--web`).
