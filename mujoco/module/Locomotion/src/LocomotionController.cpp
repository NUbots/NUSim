#include "module/Locomotion/src/LocomotionController.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "module/Locomotion/src/backends/KinematicBackend.hpp"
#include "module/Locomotion/src/backends/KinematicMath.hpp"

#ifdef K1_WITH_ONNX
#include "module/Locomotion/src/backends/PolicyBackend.hpp"
#endif

namespace k1sim::module {

namespace {

constexpr double kPi = 3.14159265358979323846;

// SDK-documented RotateHead limits (matches KinematicBackend's copy -- the
// kinematic backend applies these itself inside update(); every other mode
// that also tracks the head command (Prepare/GettingUp/LyingDown/Kicking)
// uses this helper so the clamp isn't duplicated four times).
constexpr double kHeadPitchMin = -0.3;
constexpr double kHeadPitchMax = 1.0;
constexpr double kHeadYawLimit = 0.785;

void apply_head_target(std::array<double, JOINT_COUNT>& q_ref, double pitch, double yaw) {
    q_ref[JointIndexK1::HeadPitch] = std::clamp(pitch, kHeadPitchMin, kHeadPitchMax);
    q_ref[JointIndexK1::HeadYaw]   = std::clamp(yaw, -kHeadYawLimit, kHeadYawLimit);
}

std::array<double, JOINT_COUNT> load_array(const YAML::Node& node) {
    std::array<double, JOINT_COUNT> arr{};
    if (!node) {
        return arr;
    }
    const auto values = node.as<std::vector<double>>();
    for (std::size_t i = 0; i < JOINT_COUNT && i < values.size(); ++i) {
        arr[i] = values[i];
    }
    return arr;
}

}  // namespace

LocomotionController::LocomotionController(const YAML::Node& locomotion_cfg, const YAML::Node& gains_cfg)
    : locomotion_cfg_(locomotion_cfg) {
    backend_name_        = locomotion_cfg_["backend"].as<std::string>("kinematic");
    prepare_blend_time_  = locomotion_cfg_["prepare_blend_time"].as<double>(1.0);

    const auto kinematic_node        = locomotion_cfg_["kinematic"];
    kinematic_z0_                    = kinematic_node["z0"].as<double>(0.53);
    kinematic_servo_gain_            = kinematic_node["servo_gain"].as<double>(20.0);
    kinematic_animate_               = kinematic_node["animate_steps"].as<bool>(true);
    kinematic_step_phase_gain_       = kinematic_node["step_phase_gain"].as<double>(6.0);
    kinematic_step_height_           = kinematic_node["step_height"].as<double>(0.03);

    getup_duration_    = locomotion_cfg_["getup"]["duration"].as<double>(2.0);
    liedown_duration_  = locomotion_cfg_["liedown"]["duration"].as<double>(1.5);
    kick_duration_     = locomotion_cfg_["kick"]["duration"].as<double>(0.5);

    const auto fall_node = locomotion_cfg_["fall"];
    falling_tilt_ = fall_node["falling_tilt"].as<double>(0.35);
    fallen_tilt_  = fall_node["fallen_tilt"].as<double>(1.0);
    falling_gyro_ = fall_node["falling_gyro"].as<double>(1.0);

    ready_pose_ = load_array(gains_cfg["ready_pose"]);
    pd_.kp      = load_array(gains_cfg["kp"]);
    pd_.kd      = load_array(gains_cfg["kd"]);

    if (backend_name_ == "policy") {
#ifndef K1_WITH_ONNX
        throw std::runtime_error(
            "config/locomotion.yaml sets backend: policy, but this build does not have "
            "-DK1_WITH_ONNX=ON (module::Locomotion cannot serve WALKING/SOCCER without a backend)");
#endif
    }
    else if (backend_name_ != "kinematic") {
        throw std::runtime_error("config/locomotion.yaml: unknown backend '" + backend_name_
                                  + "' (expected 'kinematic' or 'policy')");
    }
}

void LocomotionController::ensure_initialized(const mjModel* m) {
    if (initialized_) {
        return;
    }
    map_ = std::make_unique<ModelMap>(ModelMap::build(m));

    kinematic_backend_ = std::make_unique<backends::KinematicBackend>(*map_,
                                                                       pd_,
                                                                       ready_pose_,
                                                                       kinematic_z0_,
                                                                       kinematic_servo_gain_,
                                                                       kinematic_animate_,
                                                                       kinematic_step_phase_gain_,
                                                                       kinematic_step_height_,
                                                                       fallen_tilt_);

    if (backend_name_ == "policy") {
#ifdef K1_WITH_ONNX
        policy_backend_ = std::make_unique<backends::PolicyBackend>(*map_, pd_, ready_pose_, locomotion_cfg_["policy"]);
        active_backend_ = policy_backend_.get();
#else
        // Unreachable: the constructor already threw for this configuration.
        active_backend_ = kinematic_backend_.get();
#endif
    }
    else {
        active_backend_ = kinematic_backend_.get();
    }

    const int key = mj_name2id(m, mjOBJ_KEY, "lying_front");
    if (key >= 0) {
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            lying_front_pose_[i] = m->key_qpos[key * m->nq + map_->qpos_adr[i]];
        }
    }
    else {
        std::fprintf(stderr,
                      "LocomotionController: model has no 'lying_front' keyframe; "
                      "LieDown will target the ready pose instead\n");
        lying_front_pose_ = ready_pose_;
    }

    initialized_ = true;
}

std::array<double, JOINT_COUNT> LocomotionController::current_q(mjData* d) const {
    std::array<double, JOINT_COUNT> q{};
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        q[i] = d->qpos[map_->qpos_adr[i]];
    }
    return q;
}

LocomotionController::CommandSnapshot LocomotionController::snapshot_command() const {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    CommandSnapshot snap;
    snap.vx         = vx_;
    snap.vy         = vy_;
    snap.vyaw       = vyaw_;
    snap.head_pitch = head_pitch_;
    snap.head_yaw   = head_yaw_;

    snap.mode_seq        = mode_seq_;
    snap.requested_mode  = requested_mode_;
    snap.getup_seq       = getup_seq_;
    snap.getup_target    = getup_target_mode_;
    snap.liedown_seq     = liedown_seq_;
    snap.kick_seq        = kick_seq_;
    snap.kick_version    = kick_version_;

    snap.low_cmd_type   = low_cmd_type_;
    snap.low_cmd_motors = low_cmd_motors_;
    return snap;
}

void LocomotionController::set_walk_command(double vx, double vy, double vyaw) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    vx_   = vx;
    vy_   = vy;
    vyaw_ = vyaw;
}

void LocomotionController::set_head_command(double pitch, double yaw) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    head_pitch_ = pitch;
    head_yaw_   = yaw;
}

void LocomotionController::request_mode_change(int mode) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    requested_mode_ = mode;
    ++mode_seq_;
}

void LocomotionController::request_get_up(int target_mode) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    getup_target_mode_ = target_mode;
    ++getup_seq_;
}

void LocomotionController::request_lie_down() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    ++liedown_seq_;
}

void LocomotionController::request_kick(int version) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    kick_version_ = version;
    ++kick_seq_;
}

void LocomotionController::set_low_cmd(int cmd_type, std::vector<message::MotorCmdData> motors) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    low_cmd_type_   = cmd_type;
    low_cmd_motors_ = std::move(motors);
}

void LocomotionController::step(const mjModel* m, mjData* d) {
    ensure_initialized(m);
    const CommandSnapshot snap = snapshot_command();

    // Edge-triggered requests. Order matters: GetUp can interrupt anything
    // ("from any mode"), LieDown next, Kick only from a steady Walking state,
    // and a plain mode-change request last so it can still be deferred (see
    // handle_mode_request) if one of the above just started a transient.
    if (snap.getup_seq != last_getup_seq_) {
        last_getup_seq_ = snap.getup_seq;
        begin_get_up(snap.getup_target, m, d);
    }
    if (snap.liedown_seq != last_liedown_seq_) {
        last_liedown_seq_ = snap.liedown_seq;
        begin_lie_down(m, d);
    }
    if (snap.kick_seq != last_kick_seq_) {
        last_kick_seq_ = snap.kick_seq;
        if (state_ == State::Walking) {
            begin_kick(snap.kick_version, m, d);
        }
    }
    if (snap.mode_seq != last_mode_seq_) {
        last_mode_seq_ = snap.mode_seq;
        handle_mode_request(snap.requested_mode, m, d);
    }

    update_fall_detection(m, d);

    switch (state_) {
        case State::Damping: step_damping(m, d); break;
        case State::Prepare: step_prepare(m, d, snap); break;
        case State::Walking: step_walking(m, d, snap); break;
        case State::Custom: step_custom(m, d, snap); break;
        case State::GettingUp: step_getting_up(m, d); break;
        case State::LyingDown: step_lying_down(m, d, snap); break;
        case State::Kicking: step_kicking(m, d, snap); break;
    }
}

void LocomotionController::handle_mode_request(int requested_mode, const mjModel* m, mjData* d) {
    switch (requested_mode) {
        case booster::DAMPING:
        case booster::PREPARE:
        case booster::WALKING:
        case booster::CUSTOM:
        case booster::SOCCER: break;
        default:
            std::fprintf(stderr, "LocomotionController: ignoring unknown ChangeMode value %d\n", requested_mode);
            return;
    }

    if (state_ == State::GettingUp || state_ == State::LyingDown || state_ == State::Kicking) {
        // Defer: applied automatically once the transient finishes (see
        // step_getting_up/step_lying_down/step_kicking).
        state_after_transient_ = requested_mode;
        return;
    }
    enter_state_for_mode(requested_mode, m, d);
}

void LocomotionController::enter_state_for_mode(int booster_mode, const mjModel* m, mjData* d) {
    mode_.store(booster_mode, std::memory_order_relaxed);
    switch (booster_mode) {
        case booster::DAMPING: state_ = State::Damping; break;
        case booster::PREPARE:
            state_              = State::Prepare;
            prepare_start_pose_ = current_q(d);
            prepare_start_time_ = d->time;
            break;
        case booster::WALKING:
        case booster::SOCCER:
            state_ = State::Walking;
            active_backend_->reset(m, d);
            break;
        case booster::CUSTOM:
            state_                   = State::Custom;
            custom_parallel_warned_  = false;
            break;
        default: break;  // unreachable: validated by handle_mode_request
    }
}

void LocomotionController::begin_get_up(int target_mode, const mjModel* m, mjData* d) {
    state_                  = State::GettingUp;
    state_after_transient_  = target_mode;
    getting_up_.store(true, std::memory_order_relaxed);
    getup_script_.begin(m, d, *map_);
}

void LocomotionController::begin_lie_down(const mjModel* /*m*/, mjData* d) {
    state_                  = State::LyingDown;
    state_after_transient_  = booster::DAMPING;
    liedown_start_pose_     = current_q(d);
    liedown_start_time_     = d->time;
}

void LocomotionController::begin_kick(int version, const mjModel* /*m*/, mjData* d) {
    state_                  = State::Kicking;
    state_after_transient_  = mode_.load(std::memory_order_relaxed);
    kick_start_time_        = d->time;
    kick_leg_is_left_       = (version % 2 == 0);
}

void LocomotionController::step_damping(const mjModel* /*m*/, mjData* d) {
    for (int act : map_->act_id) {
        d->ctrl[act] = 0.0;
    }
}

void LocomotionController::step_prepare(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
    const double frac = std::clamp((d->time - prepare_start_time_) / prepare_blend_time_, 0.0, 1.0);
    const double s     = backends::smoothstep(frac);

    std::array<double, JOINT_COUNT> q_ref{};
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        q_ref[i] = backends::lerp(prepare_start_pose_[i], ready_pose_[i], s);
    }
    apply_head_target(q_ref, snap.head_pitch, snap.head_yaw);
    pd_.apply(m, d, *map_, q_ref);
}

void LocomotionController::step_walking(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
    const LocoCommand cmd{snap.vx, snap.vy, snap.vyaw, snap.head_pitch, snap.head_yaw};
    active_backend_->update(m, d, cmd);
}

void LocomotionController::step_custom(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
    if (snap.low_cmd_type == 0) {  // PARALLEL: unsupported, hold last ctrl
        if (!custom_parallel_warned_) {
            std::fprintf(stderr,
                          "LocomotionController: CUSTOM received cmd_type=PARALLEL, which is not "
                          "supported; holding last ctrl\n");
            custom_parallel_warned_ = true;
        }
        return;
    }

    const auto& motors  = snap.low_cmd_motors;
    const std::size_t n = std::min(motors.size(), JOINT_COUNT);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& mc = motors[i];
        const int act  = map_->act_id[i];
        const double q  = d->qpos[map_->qpos_adr[i]];
        const double dq = d->qvel[map_->dof_adr[i]];
        double tau      = mc.kp * (mc.q - q) + mc.kd * (mc.dq - dq) + mc.tau;
        if (m->actuator_forcelimited[act] != 0) {
            tau = std::clamp(tau, m->actuator_forcerange[2 * act], m->actuator_forcerange[2 * act + 1]);
        }
        d->ctrl[act] = tau;
    }
    // Joints beyond `n` (an undersized LowCmd) are left at their previous
    // ctrl value -- the same "hold" behaviour as a PARALLEL command.
}

void LocomotionController::step_getting_up(const mjModel* m, mjData* d) {
    const bool done =
        getup_script_.step(m, d, *map_, pd_, ready_pose_, getup_duration_, kinematic_servo_gain_, kinematic_z0_);
    if (done) {
        getting_up_.store(false, std::memory_order_relaxed);
        enter_state_for_mode(state_after_transient_, m, d);
    }
}

void LocomotionController::step_lying_down(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
    const double frac = std::clamp((d->time - liedown_start_time_) / liedown_duration_, 0.0, 1.0);
    const double s     = backends::smoothstep(frac);

    std::array<double, JOINT_COUNT> q_ref{};
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        q_ref[i] = backends::lerp(liedown_start_pose_[i], lying_front_pose_[i], s);
    }
    apply_head_target(q_ref, snap.head_pitch, snap.head_yaw);
    pd_.apply(m, d, *map_, q_ref);
    // Root is left passive throughout: gravity settles the base as the
    // joints curl toward the lying_front shape (then DAMPING takes over).

    if (frac >= 1.0) {
        enter_state_for_mode(state_after_transient_, m, d);
    }
}

void LocomotionController::step_kicking(const mjModel* m, mjData* d, const CommandSnapshot& snap) {
    const double frac     = std::clamp((d->time - kick_start_time_) / kick_duration_, 0.0, 1.0);
    const double envelope = std::sin(kPi * frac);  // 0 -> 1 -> 0 single swing hump

    std::array<double, JOINT_COUNT> q_ref = ready_pose_;
    constexpr double kHipAmp = 0.7, kKneeAmp = 0.9, kAnkleAmp = 0.25;
    const std::size_t hip   = kick_leg_is_left_ ? JointIndexK1::LeftHipPitch : JointIndexK1::RightHipPitch;
    const std::size_t knee  = kick_leg_is_left_ ? JointIndexK1::LeftKneePitch : JointIndexK1::RightKneePitch;
    const std::size_t ankle = kick_leg_is_left_ ? JointIndexK1::LeftAnklePitch : JointIndexK1::RightAnklePitch;
    q_ref[hip] -= kHipAmp * envelope;
    q_ref[knee] += kKneeAmp * envelope;
    q_ref[ankle] += kAnkleAmp * envelope;
    apply_head_target(q_ref, snap.head_pitch, snap.head_yaw);
    pd_.apply(m, d, *map_, q_ref);

    // Hold position (ignore any commanded vx/vy/vyaw) via the kinematic-style
    // root servo, same fallen-tilt gate as KinematicBackend.
    const double tilt = backends::base_tilt(m, d, *map_);
    if (tilt < fallen_tilt_) {
        backends::servo_root(m, d, *map_, 0.0, 0.0, kinematic_z0_, {0.0, 0.0, 1.0}, 0.0, kinematic_servo_gain_);
    }

    if (frac >= 1.0) {
        enter_state_for_mode(state_after_transient_, m, d);
    }
}

void LocomotionController::update_fall_detection(const mjModel* m, mjData* d) {
    const double tilt = backends::base_tilt(m, d, *map_);
    double gyro_mag    = 0.0;
    if (map_->sens_gyro >= 0) {
        const double* g = &d->sensordata[map_->sens_gyro];
        gyro_mag        = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
    }

    int computed = booster::IS_READY;
    if (tilt > fallen_tilt_) {
        computed = booster::HAS_FALLEN;
    }
    else if (tilt > falling_tilt_ && gyro_mag > falling_gyro_) {
        computed = booster::IS_FALLING;
    }

    fall_state_.store(state_ == State::GettingUp ? booster::IS_GETTING_UP : computed, std::memory_order_relaxed);
}

}  // namespace k1sim::module
