#ifndef K1SIM_MODULE_SKILLPLAYER_HPP
#define K1SIM_MODULE_SKILLPLAYER_HPP

#include <array>
#include <atomic>
#include <cstdint>
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

    // Plays the main robot from the keyboard with the mjlab policies (walk, receive, dribble, kick),
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
    //   R        stand the robot back up at its start
    //
    // Settings and the policy files are in config/skills.yaml; see docs/SKILL_KEYBOARD.md.
    class SkillPlayer : public NUClear::Reactor {
    public:
        explicit SkillPlayer(std::unique_ptr<NUClear::Environment> environment);
        ~SkillPlayer() override;

    private:
        enum class Phase { WAITING, BLENDING, PLAYING, FALLEN };

        // One control step's measurements, read from mjData under the sim mutex.
        struct Sample {
            std::array<double, 3> gyro{};     // trunk angular velocity, trunk frame
            std::array<double, 3> gravity{};  // unit gravity, trunk frame
            std::array<double, JOINT_COUNT> q{}, dq{};
            std::array<double, 4> ball_camera{};  // ball in the head camera frame + visible flag
            skill::World world{};
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

        void tick(const message::SimStateUpdate& state);
        void restart();
        Sample read_state();
        void key(int code, bool down);
        std::string launch_pass(const Sample& sample);
        void play(const message::SimStateUpdate& state, const Sample& sample);
        void send_targets(const std::array<double, JOINT_COUNT>& q, const skill::Gains& gains, double delay);
        std::array<float, JOINT_COUNT> run_ball_skill(skill::SkillPolicy& policy, const Sample& sample);
        std::array<float, JOINT_COUNT> run_walk(skill::SkillPolicy& policy, const Sample& sample);
        void publish_status(double sim_time, bool force, const Sample* sample = nullptr);
        skill::SkillPolicy& policy_for(const skill::Decision& decision);
        double sample_range(const Range& range);

        // -- policies --
        struct Policies;
        std::unique_ptr<Policies> policies_;

        // -- sim handles (set once, on SimHandles) --
        const mjModel* model_ = nullptr;
        mjData* data_         = nullptr;
        std::mutex* mutex_    = nullptr;
        ModelMap map_{};
        int head_body_ = -1, ball_body_ = -1, ball_geom_ = -1;
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
        // Mean change of a leg joint target between control steps (rad), since the last log line.
        double jitter_sum_ = 0.0;
        int jitter_count_  = 0;
        std::vector<ScriptEvent> script_;

        // -- state (all touched only inside Sync<SkillPlayer> reactions) --
        skill::Logic logic_;
        skill::Keys keys_{};
        bool pass_pressed_ = false, drop_pressed_ = false, release_pressed_ = false;
        bool shoot_down_ = false, shoot_up_ = false;  // Space edges since the last control step
        double view_yaw_ = 1.5708;                    // the viewer camera's default direction
        Phase phase_         = Phase::WAITING;
        uint64_t last_step_  = 0;
        uint64_t reset_seen_ = 0;
        bool have_state_     = false;
        double mode_asked_   = -1.0;  // sim time of the last CUSTOM request
        double phase_start_  = 0.0;
        std::array<double, JOINT_COUNT> blend_from_{};
        std::array<Frame, 3> history_{};
        bool history_primed_ = false;
        std::array<float, JOINT_COUNT> last_action_{};       // the ball skills' previous raw output
        std::array<float, JOINT_COUNT> walk_last_action_{};  // walk's
        std::array<double, JOINT_COUNT> last_target_{};      // joint targets last sent, whichever policy
        bool have_target_                  = false;
        const skill::SkillPolicy* running_ = nullptr;        // the policy that ran last step
        std::array<double, 4> last_ball_{0.5, 0.0, -0.8, 0.0};
        skill::Decision decision_{};
        skill::Skill logged_skill_ = skill::Skill::RECEIVE;  // last skill logged; first change is always logged
        std::size_t script_next_   = 0;
        std::string note_;  // a short message for the status line, e.g. why a key did nothing
        double note_until_  = 0.0;
        double status_sent_ = -1.0;
        std::string status_text_;
        double log_next_ = 0.0;
        std::mt19937 rng_{std::random_device{}()};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SKILLPLAYER_HPP
