#ifndef K1SIM_MODULE_SKILLPLAYER_HPP
#define K1SIM_MODULE_SKILLPLAYER_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mujoco/mujoco.h>
#include <mutex>
#include <nuclear>
#include <random>
#include <string>
#include <vector>

#include "module/SkillPlayer/src/SkillLogic.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/sim/ModelMap.hpp"

namespace k1sim::skill {
    class SkillPolicy;
    struct Gains;
}

namespace k1sim::module {

    // Plays the robots from the keyboard with the mjlab policies (walk, receive, dribble, kick),
    // inside the sim: no NUbots process and no DDS. Each 50 Hz state update it reads the robot and
    // ball from mjData, builds the running policy's observation (234 numbers for a ball skill, 75 for
    // walk), runs it and hands the 22 joint targets, with that policy's PD gains, to
    // module::Locomotion as a CUSTOM-mode servo command -- the same path a NUbots policy takes over
    // rt/joint_ctrl.
    //
    //   W A S D  without the ball: walk (W/S forward/back, A/D turn; Shift faster); walking up to a
    //            ball picks it up
    //   X        release the ball and walk
    //   Q        the ball is rolled at the robot from wherever it is, and it receives it
    //   E        a ball at rest at the robot's feet
    //   W A S D  dribble, away from / left of / towards / right of the camera (Shift: faster);
    //            nothing held = hold the ball
    //   Space    hold to wind up a shot (longer = harder, W A S D swing the aim), release to kick
    //   R        stand the robot back up at its start (rondo: start the round again)
    //
    // Training drills, from the viewer's Training buttons (SkillDrill): free play (the above, one
    // robot; the others stand on the touchline) and two rondos, where three teammates in a triangle
    // pass the ball round while the fourth, in the middle, walks at it:
    //   rondo         you pass: the keys play whichever teammate the ball is going to, a pass aims at
    //                 the most open teammate, and the computer plays the defender
    //   rondo_defend  you are the defender: the keys walk the robot in the middle, and the computer
    //                 plays the three, passing when a lane is open and dribbling away when pressed
    // A round ends when the defender's body touches the ball (bumping a robot doesn't count), or
    // when the ball rolls out of the rondo.
    //
    // Settings and the policy files are in config/skills.yaml; see docs/SKILL_KEYBOARD.md.
    class SkillPlayer : public NUClear::Reactor {
    public:
        explicit SkillPlayer(std::unique_ptr<NUClear::Environment> environment);
        ~SkillPlayer() override;

    private:
        enum class Phase { WAITING, BLENDING, PLAYING, FALLEN };
        enum class Drill { FREE, RONDO, RONDO_DEFEND };
        // PLAYER: free play's robot. TEAMMATE: one of the rondo's three passers, played whenever
        // the ball is theirs (rondo) or by the computer (rondo_defend). DEFENDER: the rondo's middle
        // robot, walking at the ball by itself (rondo) or with the keys (rondo_defend).
        // PARKED: standing on the touchline out of the way (free play's other robots).
        enum class Role { PLAYER, TEAMMATE, DEFENDER, PARKED };

        // One control step's measurements, read from mjData under the sim mutex.
        struct Sample {
            std::array<double, 3> gyro{};     // trunk angular velocity, trunk frame
            std::array<double, 3> gravity{};  // unit gravity, trunk frame
            std::array<double, JOINT_COUNT> q{}, dq{};
            std::array<double, 4> ball_camera{};  // ball in the head camera frame + visible flag
            skill::World world{};
            bool ball_contact = false;  // a geom of this robot is in contact with the ball now
            // With a touch spotted only from the ball's velocity jumping near the robot: the gap
            // (m) from the ball to the nearest part of this robot, to tell which robot touched it.
            double ball_gap = 1e9;
        };

        // One control step of every observation term; the policy sees the last three.
        struct Frame {
            std::array<float, 3> gyro{}, gravity{};
            std::array<float, JOINT_COUNT> q{}, dq{}, action{};
            std::array<float, 2> command{};
            std::array<float, 4> ball{};
        };

        struct ScriptEvent {
            double time = 0.0;
            int key     = 0;
            bool down   = false;
        };

        struct Range {
            double lo = 0.0, hi = 0.0;
        };

        // Everything about one robot: its handles in the model and its policies' running state.
        struct Agent {
            int robot_id = 1;
            std::string trunk = "Trunk";
            ModelMap map{};
            int head_body = -1;
            Role role     = Role::PLAYER;
            bool homed    = false;  // placed by a drill at `home`; a fall puts it back there
            std::array<double, 3> home{};  // x, y, yaw

            Phase phase        = Phase::WAITING;
            bool have_state    = false;
            uint64_t reset_seen = 0;
            double mode_asked  = -1.0;  // sim time of the last CUSTOM request
            double phase_start = 0.0;
            double playing_since = 0.0;
            bool start_with_ball = false;  // receive the ball at its feet once the policies take over
            std::array<double, JOINT_COUNT> blend_from{};
            std::array<Frame, 3> history{};
            bool history_primed = false;
            std::array<float, JOINT_COUNT> last_action{};       // the ball skills' previous raw output
            std::array<float, JOINT_COUNT> walk_last_action{};  // walk's
            std::array<double, JOINT_COUNT> last_target{};      // joint targets last sent, whichever policy
            bool have_target = false;
            const skill::SkillPolicy* running = nullptr;  // the policy that ran last step
            std::array<double, 4> last_ball{0.5, 0.0, -0.8, 0.0};
            std::array<double, 2> last_ball_vel{};  // for spotting a touch between control steps
            skill::Logic logic;
            skill::Decision decision{};
            skill::Skill logged_skill = skill::Skill::RECEIVE;
            double idle_since = -1.0;  // rondo: when the keys last left it
            std::array<double, 2> support{};  // rondo, off the ball: where to stand to be passed to
            bool repositioning = false, turning = false;  // support_twist's hysteresis
            double open_lane = 0.0;  // rondo: how far the defender is from its best passing lane (m)
            // rondo_defend, the computer playing it: since when it has dribbled (-1: not) and how long
            // it means to before passing, since when it has sat in receive with no pass coming, and a
            // pass key pressed last step to release this one.
            double computer_since = -1.0, computer_hold = 1.0, receiving_since = -1.0;
            bool computer_release = false;
            double log_next   = 0.0;   // sim time of its next SKILL line
            double jitter_sum = 0.0;   // mean leg target change per step since the last log line
            int jitter_count  = 0;
            Sample sample{};
        };

        void tick(const message::RobotStatesUpdate& states);
        void step_agent(Agent& agent, const message::SimStateUpdate& state);
        void restart(Agent& agent);
        Sample read_state(Agent& agent);
        void key(int code, bool down);
        skill::Keys held_keys() const;
        std::string launch_pass(Agent& agent);
        void apply_inputs(Agent& agent, double sim_time);
        void run_policy(Agent& agent, const message::SimStateUpdate& state);
        void send_targets(Agent& agent, const std::array<double, JOINT_COUNT>& q, const skill::Gains& gains, double delay);
        std::array<float, JOINT_COUNT> run_ball_skill(Agent& agent, skill::SkillPolicy& policy);
        std::array<float, JOINT_COUNT> run_walk(Agent& agent, skill::SkillPolicy& policy);
        void publish_status(double sim_time, bool force);
        skill::SkillPolicy& policy_for(const skill::Decision& decision);
        double sample_range(const Range& range);
        void note(const std::string& text, double sim_time, double seconds = 2.0);

        // -- drills --
        void start_drill(Drill drill, double sim_time);
        void start_rondo_round(double sim_time);
        void update_rondo(double sim_time);
        bool rondo() const {
            return drill_ != Drill::FREE;
        }
        bool defending() const {
            return drill_ == Drill::RONDO_DEFEND;
        }
        void set_owner(std::size_t to, double sim_time);
        void end_round(const std::string& text, double sim_time);
        bool ball_loose() const;
        bool defender_touched() const;
        skill::Keys computer_keys(Agent& agent, double sim_time);
        std::array<double, 3> chase_twist(const Agent& agent) const;
        skill::Decision defender_decision(const Agent& agent, double sim_time);
        std::array<double, 3> support_twist(Agent& agent);
        std::array<double, 3> ease_twist(const std::array<double, 3>& was, const std::array<double, 3>& want) const;
        // Caller holds the sim mutex. Stands a robot at (x, y) facing yaw, in the ready pose.
        void place_locked(Agent& agent, double x, double y, double yaw);
        void place_ball_locked(double x, double y);
        std::array<double, 2> rondo_spot(int k) const;  // teammate k's corner of the triangle

        // -- policies --
        struct Policies;
        std::unique_ptr<Policies> policies_;

        // -- sim handles (set once, on SimHandles) --
        const mjModel* model_ = nullptr;
        mjData* data_         = nullptr;
        std::mutex* mutex_    = nullptr;
        std::function<void(int)> placed_robot_;
        int ball_body_ = -1, ball_geom_ = -1;
        std::array<double, JOINT_COUNT> ready_joints_{};  // the "ready" keyframe's pose
        double ready_z_ = 0.555;
        std::array<double, 3> start_pose_{};  // free play's robot as the scene starts it: x, y, yaw
        std::array<double, 2> ball_start_{};  // and the ball
        // The scene's goals (bodies goal_pos_x / goal_neg_x with _post1 and _bar geoms), for goal
        // detection; empty on a field without them.
        struct Goal {
            double x = 0.0, y = 0.0, half_width = 0.0, height = 0.0;
            int dir = 1;
        };
        std::vector<Goal> goals_;
        bool ball_in_goal_ = false;  // latched until the ball is back in front of the goal line
        void check_goal(const Sample& sample, double sim_time);

        // -- config --
        double blend_time_       = 0.5;
        double fall_reset_after_ = 3.0;  // s on the ground before the robot is stood back up; 0 = never
        std::array<double, 3> camera_offset_{0.06, 0.0, 0.07};
        double camera_hfov_ = 1.5708, camera_vfov_ = 1.0472, camera_range_ = 5.0;
        Range pass_distance_{2.5, 3.0}, pass_bearing_{-0.3, 0.3}, pass_offset_{-0.2, 0.2}, pass_arrival_{1.5, 2.2};
        double pass_rolling_decel_ = 0.6;
        bool pass_from_ball_       = true;   // roll the pass from where the ball is, not a fresh spot
        double pass_min_distance_  = 1.0;    // m; closer than this Q does nothing (the ball is at the feet)
        double pass_trained_distance_ = 4.0;     // receive was trained on passes from up to this far...
        double pass_trained_bearing_  = 0.7854;  // ...and within this angle of the heading (rad)
        double drop_distance_      = 0.5;  // m ahead of the robot that E puts a resting ball
        double log_period_         = 1.0;  // s between SKILL log lines
        double action_delay_       = 0.025; // s between a ball skill's step and its targets reaching the servos
        double walk_action_delay_  = 0.0;   // the same for walk
        std::atomic<double>* rtf_  = nullptr;
        std::vector<ScriptEvent> script_;
        skill::Config logic_config_{};
        double touch_velocity_jump_ = 0.4;  // m/s change in one 20 ms step: more than rolling friction
        bool match_ball_contact_     = true;
        double ball_contact_friction_ = 0.3;
        std::array<double, 2> ball_contact_solref_{0.02, 0.35};

        struct Rondo {
            std::array<double, 2> centre{0.0, 0.0};
            double side            = 4.0;   // m between teammates
            double defender_speed  = 2.0;   // m/s
            double defender_turn   = 1.0;   // rad/s
            double defender_delay  = 2.0;   // s after the round starts before the defender sets off
            double defender_lead   = 1.0;   // s it looks ahead along the ball's roll, at most
            double defender_accel  = 3.0;   // m/s^2 its walking speed changes at, at most
            double defender_close_speed = 0.6;  // m/s at the ball, rising 1.5 m/s per metre away
            double support_speed     = 0.8;  // m/s teammates off the ball walk to their spot at
            double support_tolerance = 0.4;  // m from the spot before they set off again
            double lane_clear        = 0.8;  // m the defender should be off a passing lane
            double support_shift     = 1.2;  // m a teammate steps off its corner at most
            double ball_ahead      = 0.3;   // m in front of the starting teammate the ball is put
            double out_margin      = 2.0;   // m beyond the triangle's corners: the ball is lost
            double next_round      = 1.5;   // s after an interception or a lost ball
            double receive_reach   = 0.6;   // m to the side of a pass's path a teammate can still receive it
            double chase_speed     = 0.8;   // m/s a computer teammate walks to a loose ball at
            // The computer playing the three (rondo_defend):
            Range ai_hold{0.6, 1.8};        // s it keeps the ball before passing, nobody close
            double ai_pressure   = 1.6;     // m: the defender this close to the ball, it passes now...
            double ai_lane_open  = 0.7;     // m: ...if a lane is this far from the defender
            double ai_first_time = 1.8;     // m: the defender this close to a teammate, it passes first time
            std::vector<std::array<double, 2>> park{{2.5, -5.0}, {3.5, -5.0}, {4.5, -5.0}};
        } rondo_;

        // -- state (all touched only inside Sync<SkillPlayer> reactions) --
        std::vector<Agent> agents_;
        std::size_t active_ = 0;  // the agent the keys play
        std::size_t owner_  = 0;  // rondo: the teammate with the ball, or it's rolling to, or fetching it
        Drill drill_        = Drill::FREE;
        bool drill_pending_ = true;  // (re)start drill_ on the next control step
        Drill drill_wanted_ = Drill::FREE;
        int rondo_holder_   = 0;     // the teammate the next round starts with
        // passes_: completed this round, in a row; best_: the most in a round since the drill started.
        int passes_ = 0, best_ = 0, steals_ = 0, lost_ = 0;
        bool pass_live_     = false;   // a pass between teammates is rolling, not yet touched
        bool pass_handled_  = false;   // the pass leaving the owner has been given to someone
        double round_start_ = 0.0;
        double round_over_at_ = -1.0;  // sim time the next round starts; < 0 = playing
        bool defender_stopped_ = false;

        skill::Keys keys_{};
        skill::Keys masked_{};  // direction keys held as the control switched: ignored until released
        bool pass_pressed_ = false, drop_pressed_ = false, release_pressed_ = false, reset_pressed_ = false;
        bool air_down_ = false, air_up_ = false;  // C edges since the last control step
        double pass_angle_ = 0.0;                 // the way the last pass rolls (world, rad)
        bool shoot_down_ = false, shoot_up_ = false;  // Space edges since the last control step
        double view_yaw_ = 1.5708;                    // the viewer camera's default direction
        uint64_t last_step_ = 0;
        bool have_state_    = false;
        double sim_time_    = 0.0;
        std::size_t script_next_ = 0;
        std::string note_;  // a short message for the status line, e.g. why a key did nothing
        double note_until_  = 0.0;
        double status_sent_ = -1.0;
        std::string status_text_;
        std::mt19937 rng_{std::random_device{}()};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SKILLPLAYER_HPP
