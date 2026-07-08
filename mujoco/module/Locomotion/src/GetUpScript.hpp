#ifndef K1SIM_MODULE_LOCOMOTION_GETUPSCRIPT_HPP
#define K1SIM_MODULE_LOCOMOTION_GETUPSCRIPT_HPP

#include <array>
#include <mujoco/mujoco.h>

#include "shared/k1/JointIndex.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/PdController.hpp"

namespace k1sim::module {

// Scripted GetUp transient (GettingUp internal state of LocomotionController).
// From whatever pose the robot is currently in (lying_front, lying_back, or
// already standing), cubic-interpolates joint targets current -> tucked
// crouch -> ready pose over `duration` seconds; during the second half, also
// ramps the kinematic-style root servo's *target* (height and up-vector) from
// the base's actual pose at that moment to standing, so the correction is
// gradual rather than a single large-error snap (see the workstream report
// for why: a fixed always-vertical target with a ~20 s^-1 gain would demand
// ~20 rad/s corrective angular velocity from a base lying on its side).
class GetUpScript {
public:
    // Call once when a GetUpRequest is accepted.
    void begin(const mjModel* m, mjData* d, const ModelMap& map);

    // Call every physics step while GettingUp is active. Returns true once
    // the script has finished (caller should then hand control back to the
    // target steady-state mode).
    bool step(const mjModel* m,
              mjData* d,
              const ModelMap& map,
              const PdController& pd,
              const std::array<double, JOINT_COUNT>& ready_pose,
              double duration,
              double servo_gain,
              double target_z);

private:
    static std::array<double, JOINT_COUNT> crouch_pose();

    double start_time_ = 0.0;
    std::array<double, JOINT_COUNT> q_start_{};

    // If the robot is already roughly upright when GetUp begins (e.g. GetUp
    // was requested as a no-op safety call while standing), the root servo
    // engages for the *whole* duration instead of just the second half --
    // otherwise phase A's joint interpolation (current -> crouch) would let
    // gravity fold an unsupported standing robot straight to the ground
    // before phase B rights it back up. Empirically validated in the
    // workstream report: without this, the "already standing" case
    // transiently collapses to lying height before recovering; with it, the
    // base just eases smoothly toward the ready height without ever
    // dropping. lying_front/lying_back (tilt starts ~90 deg) are unaffected
    // since they fail this check and keep the original second-half-only gate.
    bool start_upright_ = false;

    // Captured once, at the moment the root servo engages (either t=0 if
    // start_upright_, or frac=0.5 otherwise).
    bool phase_b_captured_          = false;
    double phase_b_z_start_         = 0.0;
    std::array<double, 3> phase_b_up_start_{0.0, 0.0, 1.0};
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_GETUPSCRIPT_HPP
