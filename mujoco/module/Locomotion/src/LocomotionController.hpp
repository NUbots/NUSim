#ifndef K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP
#define K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "module/Locomotion/src/GetUpScript.hpp"
#include "module/Locomotion/src/LocomotionBackend.hpp"
#include "shared/k1/BoosterApi.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/PdController.hpp"
#include "shared/sim/StepController.hpp"

// LocomotionController is deliberately NUClear-free: it is exercised directly
// (no PowerPlant/Reactor) by test/unit/test_locomotion.cpp, and is driven by
// module::Simulation's physics thread in production via the StepController
// seam. All logging of *requests* (as opposed to internal FSM detail) is the
// Locomotion reactor's job (Locomotion.cpp) since only it has a NUClear
// logging context.

namespace k1sim::module {

// The Booster mode state machine (DAMPING/PREPARE/WALKING=SOCCER/CUSTOM plus
// the GettingUp/LyingDown/Kicking scripted transients). Owns the active
// LocomotionBackend and the shared PD gains / ready pose (config/gains.yaml,
// read-only here -- gains.yaml belongs to workstream B).
//
// Thread model: setters below are called from arbitrary NUClear reaction
// threads (SdkBridge's RPC dispatch) and only ever write a mutex-guarded
// mailbox. step() runs on the physics thread with the sim mutex already held
// (per StepController's contract) and snapshots the mailbox once per call;
// all FSM state (`state_`, timers, captured poses, ...) is touched only from
// step(), so it needs no synchronization of its own. mode()/fall_state()/
// getting_up() are the only cross-thread reads and are plain atomics.
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
        return getting_up_.load(std::memory_order_relaxed);
    }

    // -- thread-safe setters (called from NUClear reaction threads) --
    void set_walk_command(double vx, double vy, double vyaw);
    void set_head_command(double pitch, double yaw);
    void request_mode_change(int mode);
    void request_get_up(int target_mode);
    void request_lie_down();
    void request_kick(int version);
    void set_low_cmd(int cmd_type, std::vector<message::MotorCmdData> motors);

    // Test/introspection helpers (harmless in production: read-only).
    const LocomotionBackend* active_backend() const {
        return active_backend_;
    }
    bool is_initialized() const {
        return initialized_;
    }

private:
    enum class State { Damping, Prepare, Walking, Custom, GettingUp, LyingDown, Kicking };

    // Mailbox written by the setters above, read once per step() via a copy
    // taken under lock. `*_seq` fields are edge-triggers: step() remembers
    // the last value it consumed (last_*_seq_, physics-thread only) and acts
    // only when the snapshot's sequence has advanced, so a request is never
    // processed twice and back-to-back requests of the same kind coalesce to
    // the latest (acceptable -- these are "go to state X" requests, not a
    // queue of independent actions).
    struct CommandSnapshot {
        double vx = 0, vy = 0, vyaw = 0;
        double head_pitch = 0, head_yaw = 0;

        uint64_t mode_seq   = 0;
        int requested_mode  = booster::DAMPING;
        uint64_t getup_seq  = 0;
        int getup_target    = booster::SOCCER;
        uint64_t liedown_seq = 0;
        uint64_t kick_seq    = 0;
        int kick_version     = 1;

        int low_cmd_type = 1;
        std::vector<message::MotorCmdData> low_cmd_motors;
    };

    CommandSnapshot snapshot_command() const;

    void ensure_initialized(const mjModel* m);
    std::array<double, JOINT_COUNT> current_q(mjData* d) const;

    void handle_mode_request(int requested_mode, const mjModel* m, mjData* d);
    void enter_state_for_mode(int booster_mode, const mjModel* m, mjData* d);
    void begin_get_up(int target_mode, const mjModel* m, mjData* d);
    void begin_lie_down(const mjModel* m, mjData* d);
    void begin_kick(int version, const mjModel* m, mjData* d);

    void step_damping(const mjModel* m, mjData* d);
    void step_prepare(const mjModel* m, mjData* d, const CommandSnapshot& snap);
    void step_walking(const mjModel* m, mjData* d, const CommandSnapshot& snap);
    void step_custom(const mjModel* m, mjData* d, const CommandSnapshot& snap);
    void step_getting_up(const mjModel* m, mjData* d);
    void step_lying_down(const mjModel* m, mjData* d, const CommandSnapshot& snap);
    void step_kicking(const mjModel* m, mjData* d, const CommandSnapshot& snap);

    void update_fall_detection(const mjModel* m, mjData* d);

    // -- config (parsed once at construction; no model needed) --
    YAML::Node locomotion_cfg_;
    std::string backend_name_;
    double prepare_blend_time_;
    double kinematic_z0_, kinematic_servo_gain_;
    bool kinematic_animate_;
    double kinematic_step_phase_gain_, kinematic_step_height_;
    double getup_duration_;
    double liedown_duration_;
    double kick_duration_;
    double falling_tilt_, fallen_tilt_, falling_gyro_;

    std::array<double, JOINT_COUNT> ready_pose_{};
    PdController pd_;

    // -- lazily built on first step() (needs the mjModel) --
    bool initialized_ = false;
    std::unique_ptr<ModelMap> map_;
    std::unique_ptr<LocomotionBackend> kinematic_backend_;
    std::unique_ptr<LocomotionBackend> policy_backend_;
    LocomotionBackend* active_backend_ = nullptr;
    std::array<double, JOINT_COUNT> lying_front_pose_{};

    GetUpScript getup_script_;

    // -- FSM state (physics-thread only) --
    State state_             = State::Damping;
    int state_after_transient_ = booster::DAMPING;

    std::array<double, JOINT_COUNT> prepare_start_pose_{};
    double prepare_start_time_ = 0.0;

    std::array<double, JOINT_COUNT> liedown_start_pose_{};
    double liedown_start_time_ = 0.0;

    double kick_start_time_  = 0.0;
    bool kick_leg_is_left_   = false;

    bool custom_parallel_warned_ = false;

    uint64_t last_mode_seq_ = 0, last_getup_seq_ = 0, last_liedown_seq_ = 0, last_kick_seq_ = 0;

    // -- mailbox (guarded by cmd_mutex_) --
    mutable std::mutex cmd_mutex_;
    double vx_ = 0, vy_ = 0, vyaw_ = 0, head_pitch_ = 0, head_yaw_ = 0;
    uint64_t mode_seq_ = 0;
    int requested_mode_ = booster::DAMPING;
    uint64_t getup_seq_ = 0;
    int getup_target_mode_ = booster::SOCCER;
    uint64_t liedown_seq_ = 0;
    uint64_t kick_seq_ = 0;
    int kick_version_ = 1;
    int low_cmd_type_ = 1;
    std::vector<message::MotorCmdData> low_cmd_motors_;

    // -- cross-thread-visible state --
    std::atomic<int> mode_{booster::DAMPING};
    std::atomic<int> fall_state_{booster::IS_READY};
    std::atomic<bool> getting_up_{false};
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_LOCOMOTIONCONTROLLER_HPP
