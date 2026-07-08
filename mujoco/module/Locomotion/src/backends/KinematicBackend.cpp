#include "module/Locomotion/src/backends/KinematicBackend.hpp"

#include <algorithm>
#include <cmath>

#include "module/Locomotion/src/backends/KinematicMath.hpp"

namespace k1sim::module::backends {

namespace {
// SDK-documented RotateHead limits (see the plan's wire contract / BoosterApi.hpp).
constexpr double kHeadPitchMin = -0.3;
constexpr double kHeadPitchMax = 1.0;
constexpr double kHeadYawLimit = 0.785;

// |v| threshold below which the stepping animation is not applied (config
// "kinematic.animate_steps" gates the feature; this threshold is the
// hysteresis point from the plan: "optional procedural stepping animation
// when |v|>0.02").
constexpr double kAnimateSpeedThreshold = 0.02;

// Cosmetic-only gait shaping: maps the configured foot-clearance target
// (kinematic.step_height, ~3cm) to joint-space swing amplitudes. This is a
// coarse approximation (not inverse kinematics) tuned by inspection in the
// viewer -- the kinematic backend does not need an exact foot trajectory
// since the root is servoed directly; this only has to look reasonable and
// stay well inside each joint's range.
constexpr double kKneeGainPerMetre = 6.0;   // rad of knee bend per metre of step_height
constexpr double kHipToKneeRatio   = 0.5;
constexpr double kAnkleToKneeRatio = 0.4;
constexpr double kMaxKneeAmplitude = 1.2;   // rad, safety clamp (knee range is [0, 2.23])
constexpr double kTwoPi            = 2.0 * 3.14159265358979323846;
}  // namespace

KinematicBackend::KinematicBackend(const ModelMap& map,
                                    PdController pd,
                                    std::array<double, JOINT_COUNT> ready_pose,
                                    double z0,
                                    double servo_gain,
                                    bool animate_steps,
                                    double step_phase_gain,
                                    double step_height,
                                    double fallen_tilt)
    : map_(map)
    , pd_(std::move(pd))
    , ready_pose_(ready_pose)
    , z0_(z0)
    , servo_gain_(servo_gain)
    , animate_steps_(animate_steps)
    , step_phase_gain_(step_phase_gain)
    , step_height_(step_height)
    , fallen_tilt_(fallen_tilt) {}

void KinematicBackend::reset(const mjModel* m, mjData* d) {
    phase_       = 0.0;
    target_yaw_  = base_yaw(m, d, map_);  // start tracking from the current heading, no jump
}

void KinematicBackend::update(const mjModel* m, mjData* d, const LocoCommand& cmd) {
    std::array<double, JOINT_COUNT> q_ref = ready_pose_;

    const double speed = std::hypot(cmd.vx, cmd.vy);
    if (animate_steps_ && speed > kAnimateSpeedThreshold) {
        phase_ += m->opt.timestep * step_phase_gain_ * speed;
        if (phase_ > kTwoPi) {
            phase_ = std::fmod(phase_, kTwoPi);
        }

        const double knee_amp =
            std::clamp(step_height_ * kKneeGainPerMetre, 0.0, kMaxKneeAmplitude);
        const double hip_amp   = knee_amp * kHipToKneeRatio;
        const double ankle_amp = knee_amp * kAnkleToKneeRatio;

        // Left/right legs swing in anti-phase; knee offset is clamped to the
        // positive half-cycle so it only ever *adds* bend on top of the
        // straight-leg ready pose (the joint's range is [0, 2.23], it cannot
        // go negative).
        const double left_phase  = phase_;
        const double right_phase = phase_ + kTwoPi / 2.0;

        q_ref[JointIndexK1::LeftHipPitch] += hip_amp * std::sin(left_phase);
        q_ref[JointIndexK1::LeftKneePitch] += knee_amp * std::max(0.0, std::sin(left_phase));
        q_ref[JointIndexK1::LeftAnklePitch] -= ankle_amp * std::sin(left_phase);

        q_ref[JointIndexK1::RightHipPitch] += hip_amp * std::sin(right_phase);
        q_ref[JointIndexK1::RightKneePitch] += knee_amp * std::max(0.0, std::sin(right_phase));
        q_ref[JointIndexK1::RightAnklePitch] -= ankle_amp * std::sin(right_phase);
    }
    else {
        phase_ = 0.0;
    }

    q_ref[JointIndexK1::HeadPitch] = std::clamp(cmd.head_pitch, kHeadPitchMin, kHeadPitchMax);
    q_ref[JointIndexK1::HeadYaw]   = std::clamp(cmd.head_yaw, -kHeadYawLimit, kHeadYawLimit);

    // Gravity/bias compensation feed-forward on the two head joints only:
    // with the official kp=4 head gains, gravity alone leaves ~0.1 rad of
    // steady-state pitch error (the real robot's head servo clearly has
    // integral action / gravity comp -- the SDK's RotateHead is expected to
    // actually reach its target). qfrc_bias holds last step's
    // Coriolis+gravity bias for that dof; adding it as tau_ff makes the PD
    // track around the gravity load instead of fighting it. Deliberately NOT
    // applied to the other 20 joints: the legs are in ground contact, where
    // bias feed-forward would fight the contact forces.
    std::array<double, JOINT_COUNT> dq_ref{};
    std::array<double, JOINT_COUNT> tau_ff{};
    tau_ff[JointIndexK1::HeadPitch] = d->qfrc_bias[map_.dof_adr[JointIndexK1::HeadPitch]];
    tau_ff[JointIndexK1::HeadYaw]   = d->qfrc_bias[map_.dof_adr[JointIndexK1::HeadYaw]];

    pd_.apply(m, d, map_, q_ref, dq_ref, tau_ff);

    // Root servo: never fight the ground when fallen (a fallen robot must
    // lie until GetUp recovers it -- see the plan's fall-detection note).
    const double tilt = base_tilt(m, d, map_);
    if (tilt < fallen_tilt_) {
        const double yaw        = base_yaw(m, d, map_);
        const auto [vx_w, vy_w] = yaw_rotate(cmd.vx, cmd.vy, yaw);

        // Closed-loop yaw tracking (see the header comment on target_yaw_):
        // a bare qvel_ang_z = cmd.vyaw under-rotates while planted in ground
        // contact, so track a free-running kinematic target instead of
        // commanding the rate open-loop.
        target_yaw_ += cmd.vyaw * m->opt.timestep;
        const double yaw_err  = std::remainder(target_yaw_ - yaw, kTwoPi);
        const double vyaw_cmd = cmd.vyaw + servo_gain_ * yaw_err;

        servo_root(m, d, map_, vx_w, vy_w, z0_, {0.0, 0.0, 1.0}, vyaw_cmd, servo_gain_);
    }
}

}  // namespace k1sim::module::backends
