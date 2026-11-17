#ifndef K1SIM_MODULE_SKILLPLAYER_SKILLLOGIC_HPP
#define K1SIM_MODULE_SKILLPLAYER_SKILLLOGIC_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

// Which skill policy runs and what ball velocity it is asked for, from the operator's keys and the
// state of the world. No NUClear, MuJoCo or ONNX in here, so test/unit/test_skill_logic.cpp can
// drive it directly.
//
// The rules follow the policies' interface spec (mjlab policies/CONTROLLER_INTERFACE.md):
//   walk     body twist (vx, vy, wz): W/S forward/back, A/D turn (or step sideways). The spec trained
//            it forward to 1 m/s and found turning and sideways barely respond in mjlab; in NUSim
//            turning does, so the keys are all live here
//   receive  command (0, 0); hands over to dribble once the ball has been held still in front
//   dribble  command = the direction held, at 0.4 to 1.5 m/s, or (0, 0) to hold the ball
//   kick     one command per kick, 2 to 8 m/s, within +/-135 degrees of the robot's heading up to
//            5 m/s and +/-45 degrees above
//
// Hand-overs: walk -> receive is trained (a pass played to a walking robot) and so is dribble ->
// kick. Walking into a resting ball also hands over to receive. Releasing the ball (the release
// key) goes from any ball skill to walk, and the robot will not pick that ball up again until it
// has been at least walk_pickup_distance + pickup_rearm away from it. Getting back into walk (from a
// ball skill, once the ball is out of reach) is untested in mjlab but is the only way to go and
// fetch a ball.
//
// A shot is wound up: while the shoot key is held the robot holds the ball, the power climbs from
// kick_speed_min to kick_speed_max and the direction keys swing the aim; releasing the key kicks.
namespace k1sim::skill {

    enum class Skill { WALK, RECEIVE, DRIBBLE, KICK };

    inline const char* name(Skill skill) {
        switch (skill) {
            case Skill::WALK: return "WALK";
            case Skill::RECEIVE: return "RECEIVE";
            case Skill::DRIBBLE: return "DRIBBLE";
            case Skill::KICK: return "KICK";
        }
        return "?";
    }

    struct Config {
        // Dribble: speed asked of the ball while a direction is held, and with Shift.
        double dribble_speed      = 0.8;
        double dribble_fast_speed = 1.3;
        // What W/A/S/D are relative to. VIEW: the viewer's camera (W = away from it, A = to its left).
        // FIELD: +x, +y, -x, -y of the field. ROBOT: the robot's heading.
        enum class Frame { VIEW, FIELD, ROBOT } frame = Frame::VIEW;

        // Walk, in the robot's own frame whatever the view: W forward at walk_speed (Shift:
        // walk_fast_speed), S back at walk_back_speed. A/D turn at walk_turn_rate rad/s, or with
        // walk_side_keys = STRAFE step sideways at walk_side_speed. A rate or speed of 0 turns that off.
        double walk_speed      = 0.6;
        double walk_fast_speed = 1.0;
        double walk_back_speed = 0.3;
        double walk_turn_rate  = 0.8;
        double walk_side_speed = 0.3;
        enum class SideKeys { TURN, STRAFE } walk_side_keys = SideKeys::TURN;
        // The walk will not turn on the spot: with A/D alone it is also asked to creep forward at this
        // speed so that it steps, and so turns.
        double walk_turn_creep = 0.3;
        // There is a walk policy. Without one, W never leaves the ball skills.
        bool walk_enabled = false;
        // Start (and restart) walking, standing still until W, rather than waiting for a pass.
        bool start_walking = false;
        // While walking, a ball this close ahead of the robot hands over to receive (0 = never).
        double walk_pickup_distance = 0.8;
        // After a release, the ball must first get this much further away than walk_pickup_distance.
        double pickup_rearm = 0.4;

        // A shot's speed: kick_speed_min for a tap, rising to kick_speed_max over charge_time held.
        double kick_speed_min = 2.0;
        double kick_speed_max = 8.0;
        double charge_time    = 2.0;
        double aim_rate       = 2.0;  // rad/s the aim swings towards the direction held
        // The aim starts at this point if aim_goal, else straight ahead.
        bool aim_goal = true;
        std::array<double, 2> goal{7.0, 0.0};
        double kick_max_angle = 2.356;  // rad from the heading; behind that is untrained
        // Shots faster than this were trained only within kick_power_angle of the heading.
        double kick_power_speed = 5.0;
        double kick_power_angle = 0.785;
        double kick_reach     = 1.0;    // m; no kick is started with the ball further away
        double kick_cone      = 1.05;   // rad; or with the ball further than this off the robot's heading
        double kick_lockout   = 1.0;    // s after a kick before dribbling can resume
        // Above this ball speed the kick-out-of-a-dribble policy is used (when there is one).
        double moving_kick_speed = 0.2;

        // Receive hands over to dribble once the ball has stayed within zone_radius of `zone` (in the
        // robot's yaw frame) and slower than trap_speed for settle_time.
        std::array<double, 2> zone{0.18, 0.0};
        double zone_radius = 0.18;
        double trap_speed  = 0.3;
        double settle_time = 0.5;
        // A direction key takes over from receive early if the ball is this close and slower than
        // takeover_speed (switching while it still rolls in is untested).
        double takeover_distance = 1.0;
        double takeover_speed    = 0.8;
        // After a pass is played the robot stays in receive at least this long, whatever the keys:
        // the ball is moved a step or two later, so until then it is still where it was.
        double pass_grace = 0.3;
    };

    // Limits the policies were trained over.
    inline constexpr double DRIBBLE_SPEED_MIN = 0.4;
    inline constexpr double DRIBBLE_SPEED_MAX = 1.5;
    inline constexpr double KICK_SPEED_MIN    = 2.0;
    inline constexpr double KICK_SPEED_MAX    = 8.0;
    inline constexpr double WALK_SPEED_MAX    = 1.0;

    struct Keys {
        bool forward = false, back = false, left = false, right = false, fast = false;
        bool shoot = false;  // held: winding up a shot
        // Opposite keys cancel.
        bool any_direction() const {
            return forward != back || left != right;
        }
    };

    struct World {
        double robot_x = 0.0, robot_y = 0.0, yaw = 0.0;
        double view_yaw = 0.0;  // direction the viewer's camera looks along the ground
        bool ball_valid = false;
        double ball_x = 0.0, ball_y = 0.0, ball_z = 0.0;  // ball centre, world frame
        double ball_speed  = 0.0;
        double ball_radius = 0.0785;

        double ball_distance() const {
            return std::hypot(ball_x - robot_x, ball_y - robot_y);
        }
        // Ball position in the robot's yaw frame (x forward, y left).
        std::array<double, 2> ball_local() const {
            const double dx = ball_x - robot_x, dy = ball_y - robot_y;
            const double c = std::cos(yaw), s = std::sin(yaw);
            return {c * dx + s * dy, -s * dx + c * dy};
        }
    };

    struct Decision {
        Skill skill      = Skill::RECEIVE;
        bool moving_kick = false;                  // KICK: the ball was rolling when the kick was called
        std::array<double, 2> command_world{};     // ball velocity asked for, world frame (m/s)
        std::array<double, 2> command_local{};     // the same in the robot's yaw frame: the policy input
        std::array<double, 3> twist{};             // WALK: (vx, vy, wz) in the robot's frame
        bool aiming      = false;                  // a shot is being wound up
        double aim_angle = 0.0;                    // its direction, world frame (rad)
        double aim_speed = 0.0;                    // the speed it would be kicked at now (m/s)
    };

    inline double wrap_angle(double a) {
        return std::atan2(std::sin(a), std::cos(a));
    }

    class Logic {
    public:
        explicit Logic(Config config = {}) : cfg_(config) {
            cfg_.dribble_speed      = std::clamp(cfg_.dribble_speed, DRIBBLE_SPEED_MIN, DRIBBLE_SPEED_MAX);
            cfg_.dribble_fast_speed = std::clamp(cfg_.dribble_fast_speed, DRIBBLE_SPEED_MIN, DRIBBLE_SPEED_MAX);
            cfg_.kick_speed_min     = std::clamp(cfg_.kick_speed_min, KICK_SPEED_MIN, KICK_SPEED_MAX);
            cfg_.kick_speed_max     = std::clamp(cfg_.kick_speed_max, cfg_.kick_speed_min, KICK_SPEED_MAX);
            cfg_.walk_speed         = std::clamp(cfg_.walk_speed, 0.0, WALK_SPEED_MAX);
            cfg_.walk_fast_speed    = std::clamp(cfg_.walk_fast_speed, 0.0, WALK_SPEED_MAX);
            skill_                  = start_skill();
        }

        const Config& config() const {
            return cfg_;
        }
        Skill skill() const {
            return skill_;
        }

        // Back to the start (start-up, robot reset): standing in walk, or waiting for a pass.
        void reset() {
            skill_      = start_skill();
            settle_     = 0.0;
            kick_age_   = 0.0;
            kick_world_ = {};
            aiming_     = false;
            grace_      = 0.0;
            pickup_armed_ = true;
        }

        // A pass has just been launched at the robot, or a ball put at its feet: receive it.
        void on_pass() {
            reset();
            skill_ = Skill::RECEIVE;
            grace_ = cfg_.pass_grace;
        }

        // The shoot key going down: start winding up a shot. Returns false, changing nothing, if the
        // ball is not there to be kicked.
        bool on_shoot_press(const World& world) {
            if (skill_ == Skill::WALK || !ball_kickable(world)) {
                return false;
            }
            aim_    = cfg_.aim_goal ? std::atan2(cfg_.goal[1] - world.ball_y, cfg_.goal[0] - world.ball_x) : world.yaw;
            charge_ = 0.0;
            aim_    = clamp_aim(aim_, world.yaw);
            aiming_ = true;
            return true;
        }

        // The release key: let the ball go and walk. Returns false if already walking or there is no
        // walk policy.
        bool on_release() {
            if (!cfg_.walk_enabled || skill_ == Skill::WALK) {
                return false;
            }
            skill_         = Skill::WALK;
            aiming_        = false;
            grace_         = 0.0;
            pickup_armed_  = false;
            return true;
        }

        // The shoot key coming up: kick. Returns false if no shot was being wound up.
        bool on_shoot_release(const World& world) {
            if (!aiming_) {
                return false;
            }
            aiming_            = false;
            const double speed = aim_speed();
            aim_               = clamp_aim(aim_, world.yaw);
            kick_world_        = {speed * std::cos(aim_), speed * std::sin(aim_)};
            moving_kick_       = world.ball_speed > cfg_.moving_kick_speed;
            kick_age_          = 0.0;
            skill_             = Skill::KICK;
            return true;
        }

        // One control step of dt seconds.
        Decision tick(const Keys& keys, const World& world, double dt) {
            if (aiming_) {
                // The robot holds the ball (whichever of receive or dribble it was in, asked for zero
                // velocity) while the power climbs and the aim swings towards the direction held.
                if (!world.ball_valid || world.ball_distance() > cfg_.kick_reach) {
                    aiming_ = false;  // the ball got away
                }
                else {
                    charge_ += dt;
                    if (keys.any_direction()) {
                        const auto dir    = key_direction(keys, world);
                        const double turn = wrap_angle(std::atan2(dir[1], dir[0]) - aim_);
                        aim_ += std::clamp(turn, -cfg_.aim_rate * dt, cfg_.aim_rate * dt);
                    }
                    aim_ = clamp_aim(aim_, world.yaw);
                    Decision out;
                    out.skill     = skill_ == Skill::KICK ? Skill::DRIBBLE : skill_;
                    skill_        = out.skill;
                    out.aiming    = true;
                    out.aim_angle = aim_;
                    out.aim_speed = aim_speed();
                    return out;
                }
            }
            switch (skill_) {
                case Skill::WALK: {
                    const auto ball = world.ball_local();
                    if (!pickup_armed_ && (!world.ball_valid
                                           || world.ball_distance() > cfg_.walk_pickup_distance + cfg_.pickup_rearm)) {
                        pickup_armed_ = true;
                    }
                    if (pickup_armed_ && world.ball_valid && ball[0] > 0.0
                        && std::hypot(ball[0], ball[1]) < cfg_.walk_pickup_distance) {
                        skill_  = Skill::RECEIVE;  // walked up to the ball
                        settle_ = 0.0;
                    }
                    break;
                }
                case Skill::RECEIVE: {
                    if (grace_ > 0.0) {
                        grace_ -= dt;
                        settle_ = 0.0;
                        break;
                    }
                    const auto ball   = world.ball_local();
                    const bool in_zone = world.ball_valid && ball[0] > 0.0
                                         && std::hypot(ball[0] - cfg_.zone[0], ball[1] - cfg_.zone[1]) < cfg_.zone_radius
                                         && world.ball_z < world.ball_radius + 0.03;
                    settle_ = in_zone && world.ball_speed < cfg_.trap_speed ? settle_ + dt : 0.0;
                    const bool takeover = keys.any_direction() && world.ball_valid
                                          && world.ball_distance() < cfg_.takeover_distance
                                          && world.ball_speed < cfg_.takeover_speed;
                    if (settle_ >= cfg_.settle_time || takeover) {
                        skill_ = Skill::DRIBBLE;
                    }
                    // Not while a pass is on its way.
                    else if (wants_walk(keys, world) && world.ball_speed < cfg_.trap_speed) {
                        skill_ = Skill::WALK;
                    }
                    break;
                }
                case Skill::KICK:
                    kick_age_ += dt;
                    // The kick policy stands still afterwards. A direction key picks the dribble back up,
                    // once the swing has had time to finish and only if the ball is still around.
                    if (kick_age_ >= cfg_.kick_lockout && keys.any_direction() && world.ball_valid
                        && world.ball_distance() < cfg_.takeover_distance) {
                        skill_ = Skill::DRIBBLE;
                    }
                    else if (kick_age_ >= cfg_.kick_lockout && wants_walk(keys, world)) {
                        skill_ = Skill::WALK;
                    }
                    break;
                case Skill::DRIBBLE:
                    if (wants_walk(keys, world)) {
                        skill_ = Skill::WALK;  // the ball got away
                    }
                    break;
            }

            Decision out;
            out.skill = skill_;
            if (skill_ == Skill::WALK) {
                if (keys.forward && !keys.back) {
                    out.twist[0] = keys.fast ? cfg_.walk_fast_speed : cfg_.walk_speed;
                }
                else if (keys.back && !keys.forward) {
                    out.twist[0] = -cfg_.walk_back_speed;
                }
                if (keys.left != keys.right) {
                    const double sign = keys.left ? 1.0 : -1.0;
                    if (cfg_.walk_side_keys == Config::SideKeys::STRAFE) {
                        out.twist[1] = sign * cfg_.walk_side_speed;
                    }
                    else {
                        out.twist[2] = sign * cfg_.walk_turn_rate;
                        if (out.twist[0] == 0.0) {
                            out.twist[0] = cfg_.walk_turn_creep;
                        }
                    }
                }
                return out;
            }
            if (skill_ == Skill::DRIBBLE && keys.any_direction()) {
                const auto dir     = key_direction(keys, world);
                const double speed = keys.fast ? cfg_.dribble_fast_speed : cfg_.dribble_speed;
                out.command_world  = {speed * dir[0], speed * dir[1]};
            }
            else if (skill_ == Skill::KICK) {
                out.command_world = kick_world_;
                out.moving_kick   = moving_kick_;
            }
            const double c = std::cos(world.yaw), s = std::sin(world.yaw);
            out.command_local = {c * out.command_world[0] + s * out.command_world[1],
                                 -s * out.command_world[0] + c * out.command_world[1]};
            return out;
        }

    private:
        // Unit vector for the direction keys held, world frame. Needs keys.any_direction().
        std::array<double, 2> key_direction(const Keys& keys, const World& world) const {
            double x       = (keys.forward ? 1.0 : 0.0) - (keys.back ? 1.0 : 0.0);
            double y       = (keys.left ? 1.0 : 0.0) - (keys.right ? 1.0 : 0.0);
            const double n = std::max(std::hypot(x, y), 1e-9);
            x /= n;
            y /= n;
            const double frame = cfg_.frame == Config::Frame::VIEW    ? world.view_yaw
                                 : cfg_.frame == Config::Frame::ROBOT ? world.yaw
                                                                      : 0.0;
            const double c = std::cos(frame), s = std::sin(frame);
            return {c * x - s * y, s * x + c * y};
        }

        // The kick was trained with the ball in front of the robot (standing: 0.3 to 0.8 m ahead within
        // about 23 degrees; mid-dribble: at its feet).
        bool ball_kickable(const World& world) const {
            const auto ball = world.ball_local();
            return world.ball_valid && world.ball_distance() <= cfg_.kick_reach && ball[0] > 0.0
                   && std::abs(std::atan2(ball[1], ball[0])) <= cfg_.kick_cone;
        }

        Skill start_skill() const {
            return cfg_.walk_enabled && cfg_.start_walking ? Skill::WALK : Skill::RECEIVE;
        }

        // A direction key with the ball out of reach (or no ball at all): go and walk.
        bool wants_walk(const Keys& keys, const World& world) const {
            return cfg_.walk_enabled && keys.any_direction()
                   && (!world.ball_valid || world.ball_distance() > cfg_.takeover_distance);
        }

        // The aim, kept within the kick's trained arc about the robot's heading at the current power.
        double clamp_aim(double aim, double yaw) const {
            const double limit = aim_speed() > cfg_.kick_power_speed ? std::min(cfg_.kick_power_angle, cfg_.kick_max_angle)
                                                                     : cfg_.kick_max_angle;
            return yaw + std::clamp(wrap_angle(aim - yaw), -limit, limit);
        }

        double aim_speed() const {
            const double charged = cfg_.charge_time > 0.0 ? std::clamp(charge_ / cfg_.charge_time, 0.0, 1.0) : 1.0;
            return cfg_.kick_speed_min + charged * (cfg_.kick_speed_max - cfg_.kick_speed_min);
        }

        Config cfg_;
        Skill skill_      = Skill::RECEIVE;
        double settle_    = 0.0;
        double kick_age_  = 0.0;
        bool moving_kick_ = false;
        bool aiming_      = false;
        double aim_       = 0.0;  // world frame (rad)
        double charge_    = 0.0;  // s the shoot key has been held
        double grace_     = 0.0;  // s left of pass_grace
        bool pickup_armed_ = true;  // false after a release, until the ball is well away
        std::array<double, 2> kick_world_{};
    };

}  // namespace k1sim::skill

#endif  // K1SIM_MODULE_SKILLPLAYER_SKILLLOGIC_HPP
