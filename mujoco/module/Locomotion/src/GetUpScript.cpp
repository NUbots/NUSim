#include "module/Locomotion/src/GetUpScript.hpp"

#include <algorithm>

#include "module/Locomotion/src/backends/KinematicMath.hpp"

namespace k1sim::module {

using backends::base_tilt;
using backends::base_up_vector;
using backends::lerp;
using backends::lerp_vec3;
using backends::normalize3;
using backends::servo_root;
using backends::smoothstep;

std::array<double, JOINT_COUNT> GetUpScript::crouch_pose() {
    // Tucked-crouch mid-pose. Not motion-captured / IK-solved -- only needs
    // to (a) stay inside every joint's range and (b) look like a plausible
    // crouch, since the base itself is carried by the kinematic root servo
    // during the second half rather than by leg dynamics. See the workstream
    // report for the joint-range check.
    std::array<double, JOINT_COUNT> q{};
    q[JointIndexK1::HeadYaw]   = 0.0;
    q[JointIndexK1::HeadPitch] = 0.0;

    q[JointIndexK1::LeftShoulderPitch] = 0.2;
    q[JointIndexK1::LeftShoulderRoll]  = -1.3;
    q[JointIndexK1::LeftElbowPitch]    = -0.3;
    q[JointIndexK1::LeftElbowYaw]      = 0.0;

    q[JointIndexK1::RightShoulderPitch] = 0.2;
    q[JointIndexK1::RightShoulderRoll]  = 1.3;
    q[JointIndexK1::RightElbowPitch]    = -0.3;
    q[JointIndexK1::RightElbowYaw]      = 0.0;

    q[JointIndexK1::LeftHipPitch]   = -0.6;
    q[JointIndexK1::LeftHipRoll]    = 0.0;
    q[JointIndexK1::LeftHipYaw]     = 0.0;
    q[JointIndexK1::LeftKneePitch]  = 1.1;
    q[JointIndexK1::LeftAnklePitch] = 0.3;
    q[JointIndexK1::LeftAnkleRoll]  = 0.0;

    q[JointIndexK1::RightHipPitch]   = -0.6;
    q[JointIndexK1::RightHipRoll]    = 0.0;
    q[JointIndexK1::RightHipYaw]     = 0.0;
    q[JointIndexK1::RightKneePitch]  = 1.1;
    q[JointIndexK1::RightAnklePitch] = 0.3;
    q[JointIndexK1::RightAnkleRoll]  = 0.0;
    return q;
}

namespace {
// ~20 deg: matches the ballpark of locomotion.yaml's fall.falling_tilt, used
// here only to decide whether the root servo should hold from t=0.
constexpr double kUprightStartThreshold = 0.35;
}  // namespace

void GetUpScript::begin(const mjModel* m, mjData* d, const ModelMap& map) {
    start_time_ = d->time;
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        q_start_[i] = d->qpos[map.qpos_adr[i]];
    }
    phase_b_captured_ = false;
    start_upright_    = base_tilt(m, d, map) < kUprightStartThreshold;
}

bool GetUpScript::step(const mjModel* m,
                        mjData* d,
                        const ModelMap& map,
                        const PdController& pd,
                        const std::array<double, JOINT_COUNT>& ready_pose,
                        double duration,
                        double servo_gain,
                        double target_z) {
    const double t    = d->time - start_time_;
    const double frac = std::clamp(t / duration, 0.0, 1.0);
    const auto crouch = crouch_pose();

    std::array<double, JOINT_COUNT> q_ref{};
    if (frac < 0.5) {
        const double p = smoothstep(frac / 0.5);
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            q_ref[i] = lerp(q_start_[i], crouch[i], p);
        }
    }
    else {
        const double p = smoothstep((frac - 0.5) / 0.5);
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            q_ref[i] = lerp(crouch[i], ready_pose[i], p);
        }
    }

    // See the start_upright_ comment in the header: an already-upright start
    // holds the root servo for the whole duration (so it never free-falls
    // through the crouch phase); lying_front/lying_back only engage it for
    // the second half, exactly as before.
    if (start_upright_ || frac >= 0.5) {
        if (!phase_b_captured_) {
            phase_b_captured_ = true;
            phase_b_z_start_  = d->qpos[map.root_qpos_adr + 2];
            phase_b_up_start_ = base_up_vector(m, d, map);
        }

        const double p_root = start_upright_ ? smoothstep(frac) : smoothstep((frac - 0.5) / 0.5);
        const std::array<double, 3> target_up =
            normalize3(lerp_vec3(phase_b_up_start_, {0.0, 0.0, 1.0}, p_root));
        const double target_z_ramped = lerp(phase_b_z_start_, target_z, p_root);
        // Hold position/yaw fixed (no vx/vy/vyaw) while ramping height+upright.
        servo_root(m, d, map, 0.0, 0.0, target_z_ramped, target_up, 0.0, servo_gain);
    }

    pd.apply(m, d, map, q_ref);
    return frac >= 1.0;
}

}  // namespace k1sim::module
