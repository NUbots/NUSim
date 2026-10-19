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
#include "shared/sim/StepController.hpp"

// NUClear-free so test_locomotion.cpp can drive it directly; in production module::Simulation's
// physics thread drives it through StepController. Request logging is done by the Locomotion
// reactor.

namespace k1sim::module {

    // Reduced Booster mode state machine: DAMPING (limp), PREPARE (blend to and hold the gains.yaml
    // ready pose) and CUSTOM (PD-track rt/joint_ctrl LowCmd targets). Locomotion policies (walk,
    // get-up, kick) run on the NUbots_K1 side and arrive as LowCmd. WALKING/SOCCER requests are
    // accepted for wire compatibility but mapped to PREPARE with a warning.
    //
    // Threading: the setters run on NUClear reaction threads and only write a mutex-guarded mailbox.
    // step() runs on the physics thread with the sim mutex held, snapshots the mailbox once per call,
    // and owns all FSM state. mode()/fall_state() are atomics.
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
            return false;  // recovery is a NUbots_K1 policy
        }

        // -- thread-safe setters (called from NUClear reaction threads) --
        void set_head_command(double pitch, double yaw);
        void request_mode_change(int mode);
        void set_low_cmd(int cmd_type, std::vector<message::MotorCmdData> motors);

        // Read-only introspection for tests.
        bool is_initialized() const {
            return initialized_;
        }

    private:
        enum class State { Damping, Prepare, Custom };

        // Mailbox written by the setters above, read once per step() via a copy
        // taken under lock. `mode_seq` is an edge-trigger: step() remembers the
        // last value it consumed and acts only when the sequence has advanced.
        struct CommandSnapshot {
            double head_pitch = 0, head_yaw = 0;
            uint64_t mode_seq  = 0;
            int requested_mode = booster::DAMPING;
            int low_cmd_type   = 1;
            std::vector<message::MotorCmdData> low_cmd_motors;
        };

        CommandSnapshot snapshot_command() const;

        void ensure_initialized(const mjModel* m);
        std::array<double, JOINT_COUNT> current_q(mjData* d) const;

        void handle_mode_request(int requested_mode, mjData* d);

        void step_damping(mjData* d);
        void step_prepare(const mjModel* m, mjData* d, const CommandSnapshot& snap);
        void step_custom(const mjModel* m, mjData* d, const CommandSnapshot& snap);

        void update_fall_detection(const mjModel* m, mjData* d);

        // -- config (parsed once at construction; no model needed) --
        double prepare_blend_time_;
        double falling_tilt_, fallen_tilt_, falling_gyro_, fallen_height_;

        std::array<double, JOINT_COUNT> ready_pose_{};
        PdController pd_;

        // -- lazily built on first step() (needs the mjModel) --
        bool initialized_ = false;
        std::unique_ptr<ModelMap> map_;

        // -- FSM state (physics-thread only) --
        State state_ = State::Damping;

        std::array<double, JOINT_COUNT> prepare_start_pose_{};
        double prepare_start_time_ = 0.0;

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

        // -- cross-thread-visible state --
        std::atomic<int> mode_{booster::DAMPING};
        std::atomic<int> fall_state_{booster::IS_READY};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP
