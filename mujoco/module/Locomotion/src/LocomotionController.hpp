#ifndef K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP
#define K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/PdController.hpp"
#include "module/Locomotion/src/WalkPolicy.hpp"
#include "shared/sim/StepController.hpp"

// LocomotionController is deliberately NUClear-free: it is exercised directly
// (no PowerPlant/Reactor) by test/unit/test_locomotion.cpp, and is driven by
// module::Simulation's physics thread in production via the StepController
// seam. All logging of *requests* (as opposed to internal FSM detail) is the
// Locomotion reactor's job (Locomotion.cpp) since only it has a NUClear
// logging context.

namespace k1sim::module {

    // The reduced Booster mode state machine: DAMPING (motors limp), PREPARE
    // (blend to and hold the gains.yaml ready pose -- boot convenience, so the
    // robot doesn't collapse before a client connects) and CUSTOM (PD-track the
    // rt/joint_ctrl LowCmd servo targets). All locomotion *policies* (walking,
    // get-up, kick, ...) live on the NUbots_K1 side and arrive here as LowCmd
    // joint commands -- the sim only simulates servos and publishes state.
    // WALKING runs the built-in velocity-tracking MLP (WalkPolicy) driven by the Move
    // RPC; if no policy could be loaded (config/locomotion.yaml `walk`) it is mapped
    // to PREPARE with a warning. SOCCER is always mapped to PREPARE (NUbots_K1 owns
    // the soccer gait and drives the robot through CUSTOM).
    //
    // Thread model: setters below are called from arbitrary NUClear reaction
    // threads (SdkBridge's RPC dispatch) and only ever write a mutex-guarded
    // mailbox. step() runs on the physics thread with the sim mutex already held
    // (per StepController's contract) and snapshots the mailbox once per call;
    // all FSM state is touched only from step(). mode()/fall_state() are the only
    // cross-thread reads and are plain atomics.
    class LocomotionController : public StepController {
    public:
        // locomotion_cfg: config/locomotion.yaml root node.
        // gains_cfg: config/gains.yaml root node (kp/kd/ready_pose, JointIndexK1 order).
        LocomotionController(const YAML::Node& locomotion_cfg, const YAML::Node& gains_cfg);

        // -- StepController --
        void step(const mjModel* m, mjData* d) override;
        int mode() const override {
            return mode_.load(std::memory_order_relaxed);
        }
        int fall_state() const override {
            return fall_state_.load(std::memory_order_relaxed);
        }
        bool getting_up() const override {
            return false;  // scripted get-up removed; recovery is a NUbots_K1 policy
        }

        // -- thread-safe setters (called from NUClear reaction threads) --
        void set_head_command(double pitch, double yaw);
        void request_mode_change(int mode);
        void set_low_cmd(int cmd_type, std::vector<message::MotorCmdData> motors);
        void set_walk_command(double vx, double vy, double vyaw);

        bool walk_policy_loaded() const {
            return walk_policy_ != nullptr;
        }

        // Test/introspection helper (harmless in production: read-only).
        bool is_initialized() const {
            return initialized_;
        }

    private:
        enum class State { Damping, Prepare, Custom, Walking };

        // Mailbox written by the setters above, read once per step() via a copy
        // taken under lock. `mode_seq` is an edge-trigger: step() remembers the
        // last value it consumed and acts only when the sequence has advanced.
        struct CommandSnapshot {
            double head_pitch = 0, head_yaw = 0;
            uint64_t mode_seq  = 0;
            int requested_mode = booster::DAMPING;
            int low_cmd_type   = 1;
            std::vector<message::MotorCmdData> low_cmd_motors;
            std::array<double, 3> walk_cmd{};
        };

        CommandSnapshot snapshot_command() const;

        void ensure_initialized(const mjModel* m);
        std::array<double, JOINT_COUNT> current_q(mjData* d) const;

        void handle_mode_request(int requested_mode, mjData* d);

        void step_damping(mjData* d);
        void step_prepare(const mjModel* m, mjData* d, const CommandSnapshot& snap);
        void step_custom(const mjModel* m, mjData* d, const CommandSnapshot& snap);

        void step_walking(const mjModel* m, mjData* d, const CommandSnapshot& snap);

        void update_fall_detection(const mjModel* m, mjData* d);

        // -- config (parsed once at construction; no model needed) --
        double prepare_blend_time_;
        double falling_tilt_, fallen_tilt_, falling_gyro_, fallen_height_;

        std::array<double, JOINT_COUNT> ready_pose_{};
        PdController pd_;

        // -- walk policy (null if `walk.policy` is unset or failed to load) --
        std::unique_ptr<WalkPolicy> walk_policy_;
        PdController walk_pd_;  // the policy's own training gains
        double walk_blend_time_ = 1.5;
        double walk_period_     = 0.02;
        std::array<double, 3> walk_cmd_limit_{1.0, 0.5, 1.0};

        // -- lazily built on first step() (needs the mjModel) --
        bool initialized_ = false;
        std::unique_ptr<ModelMap> map_;

        // -- FSM state (physics-thread only) --
        State state_ = State::Damping;

        std::array<double, JOINT_COUNT> prepare_start_pose_{};
        double prepare_start_time_ = 0.0;

        std::array<double, JOINT_COUNT> walk_start_pose_{}, walk_q_ref_{};
        double walk_start_time_ = 0.0, walk_next_tick_ = 0.0;
        bool walk_running_      = false;  // false while blending to the policy's default pose
        std::array<double, JOINT_COUNT> q_lo_{}, q_hi_{};  // joint limits (+-inf if unlimited)

        bool custom_parallel_warned_ = false;
        // PD-hold target for a CUSTOM session that has not received a LowCmd yet
        std::array<double, JOINT_COUNT> custom_entry_pose_{};
        bool custom_low_cmd_seen_ = false;
        bool custom_entry_step_   = false;  // the entry step's snapshot predates the mailbox clear

        uint64_t last_mode_seq_ = 0;

        // -- mailbox (guarded by cmd_mutex_) --
        mutable std::mutex cmd_mutex_;
        double head_pitch_ = 0, head_yaw_ = 0;
        uint64_t mode_seq_  = 0;
        int requested_mode_ = booster::DAMPING;
        int low_cmd_type_   = 1;
        std::vector<message::MotorCmdData> low_cmd_motors_;
        std::array<double, 3> walk_cmd_{};

        // -- cross-thread-visible state --
        std::atomic<int> mode_{booster::DAMPING};
        std::atomic<int> fall_state_{booster::IS_READY};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP
