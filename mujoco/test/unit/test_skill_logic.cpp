// module::SkillPlayer's skill-switching rules (SkillLogic.hpp), with no sim, policies or keyboard:
//   A. receive hands over to dribble only after the ball has sat still in the trap zone for settle_time
//   B. a direction key takes over from receive early, but only with the ball close
//   C. dribble commands: field, robot and camera-view frames, Shift speed, opposite keys cancel,
//      yaw-frame rotation
//   D. shoot: refused out of reach; the ball is held while Space is down; power climbs with the hold;
//      direction keys swing the aim; clamped to 135 degrees off the heading; a rolling ball selects
//      the moving kick; the wind-up is dropped if the ball gets away
//   E. after a kick, a direction key resumes dribbling only once the lockout has passed
//   F. a new pass returns to receive
#include <cmath>
#include <cstdio>

#include "module/SkillPlayer/src/SkillLogic.hpp"

namespace {

    using namespace k1sim::skill;

    int failures = 0;

    void check(bool ok, const char* what) {
        if (!ok) {
            std::printf("FAIL: %s\n", what);
            ++failures;
        }
    }

    bool near(double a, double b, double tolerance = 1e-6) {
        return std::abs(a - b) < tolerance;
    }

    constexpr double DT = 0.02;

    World world_with_ball(double ball_x, double ball_y, double speed = 0.0, double yaw = 0.0) {
        World w;
        w.yaw        = yaw;
        w.ball_valid = true;
        w.ball_x     = ball_x;
        w.ball_y     = ball_y;
        w.ball_z     = w.ball_radius;
        w.ball_speed = speed;
        return w;
    }

    Decision run(Logic& logic, const Keys& keys, const World& world, double seconds) {
        Decision d;
        for (int i = 0; i < static_cast<int>(std::lround(seconds / DT)); ++i) {
            d = logic.tick(keys, world, DT);
        }
        return d;
    }

}  // namespace

int main() {
    const double pi = std::acos(-1.0);

    {  // A
        Logic logic;
        check(run(logic, {}, world_with_ball(2.0, 0.0), 1.0).skill == Skill::RECEIVE, "A: far ball stays in receive");
        check(run(logic, {}, world_with_ball(0.2, 0.0, 1.0), 1.0).skill == Skill::RECEIVE,
              "A: a ball still rolling in the zone is not trapped");
        check(run(logic, {}, world_with_ball(0.2, 0.0), 0.3).skill == Skill::RECEIVE, "A: not before settle_time");
        const Decision d = run(logic, {}, world_with_ball(0.2, 0.0), 0.3);
        check(d.skill == Skill::DRIBBLE, "A: trapped ball hands over to dribble");
        check(d.command_world[0] == 0.0 && d.command_world[1] == 0.0, "A: dribble with no key holds the ball");
    }

    {  // B
        Keys forward;
        forward.forward = true;
        Logic far;
        check(far.tick(forward, world_with_ball(2.0, 0.0), DT).skill == Skill::RECEIVE, "B: no takeover at range");
        Logic close;
        check(close.tick(forward, world_with_ball(0.6, 0.0, 0.5), DT).skill == Skill::DRIBBLE, "B: takeover up close");
    }

    {  // C
        Keys left;
        left.left = true;
        Config field_frame;
        field_frame.frame = Config::Frame::FIELD;
        Logic field(field_frame);
        const World turned = world_with_ball(0.5, 0.0, 0.0, pi / 2.0);  // robot facing +y
        Decision d         = field.tick(left, turned, DT);
        check(d.skill == Skill::DRIBBLE && near(d.command_world[0], 0.0) && near(d.command_world[1], 0.8),
              "C: field frame, A = +y at the dribble speed");
        check(near(d.command_local[0], 0.8) && near(d.command_local[1], 0.0),
              "C: +y in the field is straight ahead for a robot facing +y");

        Config robot_frame;
        robot_frame.frame = Config::Frame::ROBOT;
        Logic robot(robot_frame);
        d = robot.tick(left, turned, DT);
        check(near(d.command_world[0], -0.8) && near(d.command_world[1], 0.0), "C: robot frame, A = the robot's left");

        // The default: relative to the camera. Looking along +y (the viewer's start), W goes to +y
        // and D to +x; with the camera turned to look along -x, W goes to -x.
        Logic view;
        World seen    = world_with_ball(0.5, 0.0);
        seen.view_yaw = pi / 2.0;
        Keys forward, right;
        forward.forward = true;
        right.right     = true;
        d               = view.tick(forward, seen, DT);
        check(near(d.command_world[0], 0.0) && near(d.command_world[1], 0.8), "C: view frame, W = away from the camera");
        d = view.tick(right, seen, DT);
        check(near(d.command_world[0], 0.8) && near(d.command_world[1], 0.0), "C: view frame, D = the camera's right");
        seen.view_yaw = pi;
        d             = view.tick(forward, seen, DT);
        check(near(d.command_world[0], -0.8) && near(d.command_world[1], 0.0), "C: view frame follows the camera");

        Keys fast = left;
        fast.fast = true;
        d         = field.tick(fast, turned, DT);
        check(near(std::hypot(d.command_world[0], d.command_world[1]), 1.8), "C: Shift uses the fast speed");

        Keys both;
        both.forward = both.back = true;
        d                        = field.tick(both, turned, DT);
        check(d.command_world[0] == 0.0 && d.command_world[1] == 0.0, "C: opposite keys cancel to a hold");

        Config too_fast;
        too_fast.dribble_speed = 3.0;
        check(near(Logic(too_fast).config().dribble_speed, DRIBBLE_SPEED_MAX), "C: speed clamped to the trained range");
    }

    {  // D
        Config field_frame;
        field_frame.frame          = Config::Frame::FIELD;
        field_frame.kick_speed_max   = 5.0;
        field_frame.charge_time      = 1.5;
        field_frame.kick_power_speed = 5.0;  // these checks are of the aim, not the power limit
        Logic logic(field_frame);
        check(!logic.on_shoot_press(world_with_ball(3.0, 0.0)) && !logic.on_shoot_release(world_with_ball(3.0, 0.0))
                  && logic.skill() == Skill::RECEIVE,
              "D: no shot with the ball out of reach");

        // A tap: minimum power, at the goal.
        const World w = world_with_ball(0.5, 0.0);
        check(logic.on_shoot_press(w), "D: wind-up accepted");
        Decision d = logic.tick({}, w, DT);
        check(d.aiming && d.skill != Skill::KICK && d.command_world[0] == 0.0, "D: the ball is held while winding up");
        logic.on_shoot_release(w);
        d = logic.tick({}, w, DT);
        check(d.skill == Skill::KICK && !d.moving_kick && near(d.command_world[0], 2.0, 0.05)
                  && near(d.command_world[1], 0.0),
              "D: a tap is a minimum-power shot at the goal");

        // Held: power climbs with time and stops at the maximum.
        logic.on_shoot_press(w);
        d = run(logic, {}, w, 0.75);
        check(near(d.aim_speed, 3.5, 0.05), "D: half the charge time gives half the range");
        d = run(logic, {}, w, 3.0);
        check(near(d.aim_speed, 5.0), "D: power stops at the maximum");

        // A direction held swings the aim towards it at aim_rate, and the shot goes where it points.
        Keys left;
        left.left = true;
        d         = run(logic, left, w, 0.25);
        check(near(d.aim_angle, 0.5, 0.05), "D: the aim swings at aim_rate towards the direction held");
        d = run(logic, left, w, 2.0);
        check(near(d.aim_angle, pi / 2.0, 1e-3), "D: and stops on that direction");
        logic.on_shoot_release(world_with_ball(0.5, 0.0, 0.6));
        d = logic.tick({}, world_with_ball(0.5, 0.0, 0.6), DT);
        check(d.skill == Skill::KICK && near(d.command_world[0], 0.0, 1e-3) && near(d.command_world[1], 5.0, 1e-3),
              "D: the shot goes along the aim at the charged speed");
        check(d.moving_kick, "D: a rolling ball selects the moving kick");

        // Facing away from the goal: the aim stops 135 degrees off the heading.
        const World away = world_with_ball(-0.5, 0.0, 0.0, pi);
        logic.on_shoot_press(away);
        logic.on_shoot_release(away);
        d = logic.tick({}, away, DT);
        check(near(std::abs(std::atan2(d.command_local[1], d.command_local[0])), 2.356, 1e-3),
              "D: aim clamped to 135 degrees off the heading");

        // Not with the ball beside or behind the robot.
        logic.on_drop();
        check(!logic.on_shoot_press(world_with_ball(0.0, 0.8)) && !logic.on_shoot_press(world_with_ball(-0.5, 0.0)),
              "D: no shot with the ball beside or behind the robot");

        // The ball rolling away cancels the wind-up.
        logic.on_drop();
        logic.on_shoot_press(w);
        d = logic.tick({}, world_with_ball(3.0, 0.0), DT);
        check(!d.aiming && !logic.on_shoot_release(w), "D: the wind-up is dropped if the ball gets away");
    }

    {  // D2: above 5 m/s the aim stays within 45 degrees of the heading
        Config power;
        power.frame = Config::Frame::FIELD;
        Logic logic(power);
        check(near(logic.config().kick_speed_max, 7.0), "D2: shots go up to 7 m/s");
        Keys left;
        left.left     = true;
        const World w = world_with_ball(0.5, 0.0);
        logic.on_shoot_press(w);
        Decision d = run(logic, left, w, 0.5);  // 3.5 m/s: the aim may swing past 45 degrees
        check(d.aim_speed < 4.5 && d.aim_angle > 0.8, "D2: below 4.5 m/s the aim swings wide");
        d = run(logic, left, w, 2.0);  // full power
        check(near(d.aim_speed, 7.0) && near(d.aim_angle, 0.785, 1e-3), "D2: at full power it is held to 45 degrees");
        logic.on_shoot_release(w);
        d = logic.tick({}, w, DT);
        check(near(std::atan2(d.command_world[1], d.command_world[0]), 0.785, 1e-3)
                  && near(std::hypot(d.command_world[0], d.command_world[1]), 7.0),
              "D2: and kicked along it");
    }

    {  // E
        Logic logic;
        Keys forward;
        forward.forward = true;
        const World w   = world_with_ball(0.5, 0.0);
        logic.on_shoot_press(w);
        logic.on_shoot_release(w);
        check(run(logic, forward, w, 0.5).skill == Skill::KICK, "E: still kicking inside the lockout");
        check(run(logic, forward, w, 0.6).skill == Skill::DRIBBLE, "E: a direction key resumes the dribble after it");

        logic.on_shoot_press(w);
        logic.on_shoot_release(w);
        check(run(logic, forward, world_with_ball(5.0, 0.0), 2.0).skill == Skill::KICK,
              "E: no dribble once the ball has gone");
    }

    {  // F
        Logic logic;
        logic.on_shoot_press(world_with_ball(0.5, 0.0));
        logic.on_shoot_release(world_with_ball(0.5, 0.0));
        logic.on_drop();
        check(logic.tick({}, world_with_ball(2.5, 0.0, 2.0), DT).skill == Skill::RECEIVE, "F: a pass returns to receive");
    }

    {  // G: walk
        Config walking;
        walking.walk_enabled  = true;
        walking.start_walking = true;
        Logic logic(walking);
        check(logic.skill() == Skill::WALK, "G: starts walking");
        const World far = world_with_ball(5.0, 0.0);
        Decision d      = logic.tick({}, far, DT);
        check(d.skill == Skill::WALK && d.twist[0] == 0.0, "G: no key stands still");
        Keys forward;
        forward.forward = true;
        d               = logic.tick(forward, far, DT);
        check(near(d.twist[0], 0.6) && d.twist[1] == 0.0 && d.twist[2] == 0.0, "G: W walks forward at walk_speed");
        Keys fast = forward;
        fast.fast = true;
        check(near(logic.tick(fast, far, DT).twist[0], 1.0), "G: Shift walks at walk_fast_speed");
        Keys left;
        left.left = true;
        check(near(logic.tick(left, far, DT).twist[2], 0.8), "G: A turns left");
        Keys right, back;
        right.right = true;
        back.back   = true;
        check(near(logic.tick(right, far, DT).twist[2], -0.8), "G: D turns right");
        check(near(logic.tick(back, far, DT).twist[0], -0.3), "G: S walks back");
        check(!logic.on_shoot_press(world_with_ball(0.9, 0.0)), "G: no shot while walking");

        // Walking up to a ball ahead hands over to receive; one behind does not.
        check(logic.tick(forward, world_with_ball(-0.5, 0.0), DT).skill == Skill::WALK, "G: a ball behind is ignored");
        check(logic.tick(forward, world_with_ball(0.6, 0.0), DT).skill == Skill::RECEIVE, "G: walking into a ball picks it up");

        // A pass from walk goes to receive, and holds there briefly whatever the keys.
        logic.reset();
        logic.on_drop();
        check(logic.tick(forward, world_with_ball(0.3, 0.0), DT).skill == Skill::RECEIVE, "G: a pass is received");
        // Back to walk with W once the ball is out of reach, not while a pass rolls in.
        run(logic, {}, world_with_ball(3.0, 0.0, 2.0), 0.5);
        check(logic.tick(forward, world_with_ball(3.0, 0.0, 2.0), DT).skill == Skill::RECEIVE,
              "G: W does not walk away from an incoming pass");
        check(logic.tick(forward, world_with_ball(3.0, 0.0), DT).skill == Skill::WALK, "G: W walks to a far, still ball");

        Config strafing = walking;
        strafing.walk_side_keys = Config::SideKeys::STRAFE;
        Logic strafer(strafing);
        const Decision st = strafer.tick(left, far, DT);
        check(near(st.twist[1], 0.3) && st.twist[2] == 0.0, "G: with strafe, A steps left");

        // Any direction key with the ball far walks, not just W.
        Logic idle(walking);
        idle.on_drop();
        run(idle, {}, world_with_ball(3.0, 0.0), 0.5);
        check(idle.tick(left, world_with_ball(3.0, 0.0), DT).skill == Skill::WALK, "G: A walks to a far, still ball");

        // Release: from dribble straight to walk, and the ball is not picked straight back up.
        Logic holder(walking);
        holder.on_drop();
        const World at_feet = world_with_ball(0.2, 0.0);
        run(holder, {}, at_feet, 1.0);
        check(holder.skill() == Skill::DRIBBLE, "G: trapped, dribbling");
        check(holder.on_release() && holder.skill() == Skill::WALK, "G: X releases the ball into walk");
        check(run(holder, forward, at_feet, 0.5).skill == Skill::WALK, "G: the released ball is not picked straight up");
        run(holder, forward, world_with_ball(1.5, 0.0), 0.1);  // walked away from it
        check(holder.tick(forward, world_with_ball(0.6, 0.0), DT).skill == Skill::RECEIVE,
              "G: once well away, walking into it picks it up again");
        check(!holder.on_release() || holder.skill() == Skill::WALK, "G: release from receive walks too");
        check(!holder.on_release(), "G: nothing to release while walking");
    }

    {  // G2: without a walk policy W never walks
        Config no_walk;
        no_walk.start_walking = true;
        Logic logic(no_walk);
        Keys forward;
        forward.forward = true;
        check(logic.skill() == Skill::RECEIVE && logic.tick(forward, world_with_ball(5.0, 0.0), DT).skill == Skill::RECEIVE,
              "G2: no walk policy, no walking");
    }

    {  // I: a pass rolling in. It comes from +x towards the robot, so "back where it came from" is +x.
        Config cfg;
        cfg.frame               = Config::Frame::FIELD;
        cfg.first_touch_enabled = cfg.first_pass_enabled = cfg.air_enabled = cfg.walk_enabled = true;
        const World coming = world_with_ball(2.0, 0.0, 2.0);  // 2 m away, rolling in
        World touch        = world_with_ball(0.2, 0.0, 1.5);
        touch.ball_touched = true;
        Keys none, left, right, back;
        left.left   = true;
        right.right = true;
        back.back   = true;

        // Nothing pressed: an ordinary receive, and Q still means a first-time pass until the touch.
        Logic plain(cfg);
        plain.on_pass(M_PI);
        Decision d = run(plain, none, coming, 0.5);
        check(d.skill == Skill::RECEIVE && d.pass_incoming && plain.pass_incoming(), "I: no key, it is received");
        d = plain.tick(none, touch, DT);
        check(d.skill == Skill::RECEIVE && !plain.pass_incoming(), "I: after the touch it is an ordinary receive");

        // A direction before the touch loads a first touch, kept within 90 degrees of where it came from.
        Logic touchy(cfg);
        touchy.on_pass(M_PI);
        run(touchy, none, coming, 0.4);  // past the grace
        d = touchy.tick(left, coming, DT);
        check(d.skill == Skill::FIRST_TOUCH && near(d.command_world[0], 0.0, 1e-9) && near(d.command_world[1], 0.9),
              "I: A loads a first touch to the left at the first-touch speed");
        d = touchy.tick(back, coming, DT);  // straight on, past the robot: not allowed
        check(near(std::abs(std::atan2(d.command_world[1], d.command_world[0])), M_PI / 2, 1e-6),
              "I: a touch on past the robot is held to 90 degrees");
        d = touchy.tick(none, coming, DT);
        check(d.skill == Skill::FIRST_TOUCH && near(std::hypot(d.command_world[0], d.command_world[1]), 0.9),
              "I: releasing the key keeps the loaded touch");
        d = touchy.tick(none, touch, DT);
        check(d.touched && d.skill == Skill::FIRST_TOUCH, "I: the touch is seen");
        d = run(touchy, left, world_with_ball(0.6, 0.0, 0.9), 0.4);
        check(d.skill == Skill::DRIBBLE, "I: the dribble takes over after the touch window");
        World turned       = world_with_ball(0.6, 0.0, 0.9, 1.0);  // robot (and view) turned since
        turned.view_yaw    = 1.0;
        d = touchy.tick(left, turned, DT);
        // The touch was last loaded with S (held to -90 degrees), so it went to -y.
        check(near(d.command_world[0], 0.0, 1e-9) && near(d.command_world[1], -0.8),
              "I: with the same key held it dribbles on along the touch, whatever the view does");
        d = touchy.tick(back, turned, DT);  // field frame: S is -x
        check(near(d.command_world[0], -0.8) && near(d.command_world[1], 0.0, 1e-9), "I: a new key steers as usual");

        // Q while it comes: a first-time pass, straight back with no key, at the first-pass speed.
        Logic passer(cfg);
        check(!passer.on_first_pass(none, coming), "I: no first-time pass without a pass coming");
        passer.on_pass(M_PI);
        run(passer, none, coming, 0.4);
        check(passer.on_first_pass(none, coming), "I: Q with a pass coming plays it first time");
        d = passer.tick(none, coming, DT);
        check(d.skill == Skill::FIRST_PASS && near(d.command_world[0], 3.0) && near(d.command_world[1], 0.0, 1e-9),
              "I: first-time pass straight back at 3 m/s");
        d = passer.tick(right, coming, DT);
        check(near(d.command_world[1], -3.0) && near(d.command_world[0], 0.0, 1e-9), "I: D aims it to the right");
        passer.tick(none, touch, DT);
        check(run(passer, right, world_with_ball(3.0, 0.0, 3.0), 0.5).skill == Skill::FIRST_PASS,
              "I: it stands still after the pass");
        check(run(passer, right, world_with_ball(3.0, 0.0, 3.0), 0.6).skill == Skill::WALK, "I: then walks off");

        // Never reached: back to receiving.
        Logic missed(cfg);
        missed.on_pass(M_PI);
        run(missed, none, coming, 0.4);
        missed.tick(left, coming, DT);
        check(run(missed, none, world_with_ball(3.0, 2.0), 4.0).skill == Skill::RECEIVE, "I: no touch, back to receive");

        // The air shot: its own key, its own kind of kick; the other key's release doesn't fire it.
        Logic air(cfg);
        const World w = world_with_ball(0.5, 0.0);
        air.on_drop();
        check(air.on_shoot_press(w, true), "I: C winds up an air shot");
        check(!air.on_shoot_release(w, false), "I: Space's release doesn't fire it");
        check(air.on_shoot_release(w, true), "I: C's release does");
        d = air.tick(none, w, DT);
        check(d.skill == Skill::KICK && d.air, "I: and it is an air kick");
        Config no_air = cfg;
        no_air.air_enabled = false;
        Logic ground(no_air);
        ground.on_drop();
        check(!ground.on_shoot_press(w, true), "I: no air shot without its policy");
    }

    {  // H: a fast incoming ball is not taken over by a direction key
        Logic logic;
        Keys forward;
        forward.forward = true;
        check(logic.tick(forward, world_with_ball(0.6, 0.0, 2.0), DT).skill == Skill::RECEIVE,
              "H: no takeover while the ball still rolls in");
    }

    if (failures == 0) {
        std::printf("test_skill_logic: all checks passed\n");
    }
    return failures == 0 ? 0 : 1;
}
