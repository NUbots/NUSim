#ifndef K1SIM_MODULE_LOCOMOTION_LOCOMOTIONBACKEND_HPP
#define K1SIM_MODULE_LOCOMOTION_LOCOMOTIONBACKEND_HPP

#include <mujoco/mujoco.h>

// FROZEN INTERFACE (M0): both locomotion backends and the mode machine build
// against this. Changes require lead coordination across workstreams.

namespace k1sim {

struct LocoCommand {
    double vx = 0.0, vy = 0.0, vyaw = 0.0;      // body-frame planar velocity (SDK Move semantics)
    double head_pitch = 0.0, head_yaw = 0.0;    // RotateHead targets (pitch down-positive)
};

class LocomotionBackend {
public:
    virtual ~LocomotionBackend() = default;

    // Called on entry to WALKING/SOCCER (and after a GetUp completes).
    virtual void reset(const mjModel* m, mjData* d) = 0;

    // Called EVERY physics step before mj_step, on the physics thread with the
    // sim mutex held. Writes d->ctrl torques for all 22 actuators (including
    // head PD to cmd.head_*). The kinematic backend additionally writes the
    // root free-joint qvel.
    virtual void update(const mjModel* m, mjData* d, const LocoCommand& cmd) = 0;

    // false => the root is velocity-servoed (non-physical locomotion)
    virtual bool is_dynamic() const = 0;

    virtual const char* name() const = 0;
};

}  // namespace k1sim

#endif  // K1SIM_MODULE_LOCOMOTION_LOCOMOTIONBACKEND_HPP
