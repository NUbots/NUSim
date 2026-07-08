#ifndef K1SIM_MODULE_LOCOMOTION_BACKENDS_KINEMATICBACKEND_HPP
#define K1SIM_MODULE_LOCOMOTION_BACKENDS_KINEMATICBACKEND_HPP

#include <array>

#include "module/Locomotion/src/LocomotionBackend.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/PdController.hpp"

namespace k1sim::module::backends {

// Default, non-dynamic locomotion backend (see the plan's "Kinematic
// backend" section): every step it overwrites the 6 root free-joint qvel
// entries with a velocity servo (planar velocity from the Move command,
// height + upright servo toward config z0, yaw rate from the command) and
// PD-drives the 22 joints to the ready pose plus an optional procedural
// stepping animation. MuJoCo integrates qpos and resolves contacts normally,
// so the IMU stays smooth and the ball can still be dribbled/kicked via real
// contact -- only the root's velocity is "cheated".
class KinematicBackend : public LocomotionBackend {
public:
    KinematicBackend(const ModelMap& map,
                      PdController pd,
                      std::array<double, JOINT_COUNT> ready_pose,
                      double z0,
                      double servo_gain,
                      bool animate_steps,
                      double step_phase_gain,
                      double step_height,
                      double fallen_tilt);

    void reset(const mjModel* m, mjData* d) override;
    void update(const mjModel* m, mjData* d, const LocoCommand& cmd) override;
    bool is_dynamic() const override {
        return false;
    }
    const char* name() const override {
        return "kinematic";
    }

private:
    const ModelMap& map_;
    PdController pd_;
    std::array<double, JOINT_COUNT> ready_pose_;
    double z0_;
    double servo_gain_;
    bool animate_steps_;
    double step_phase_gain_;  // rad of gait phase per metre of commanded travel
    double step_height_;      // m, target foot clearance during swing
    double fallen_tilt_;      // rad; root servo disengages at/above this tilt

    double phase_ = 0.0;  // gait phase accumulator, only advances while moving

    // Closed-loop yaw tracking -- see the .cpp for why this exists: unlike
    // vx/vy/height/tilt, a bare `qvel_ang_z = cmd.vyaw` open-loop command
    // measurably under-rotates while the feet are planted in ground contact
    // (verified empirically; not a sliding-friction effect -- it persists
    // even at near-zero friction, and vanishes entirely with the feet
    // airborne). target_yaw_ is a free-running kinematic reference
    // (integrates cmd.vyaw exactly); the servo then commands
    // cmd.vyaw + servo_gain_*(target_yaw_ - actual_yaw) instead of bare
    // cmd.vyaw, the same closed-loop-to-a-moving-target pattern already used
    // for height/upright.
    double target_yaw_ = 0.0;
};

}  // namespace k1sim::module::backends

#endif  // K1SIM_MODULE_LOCOMOTION_BACKENDS_KINEMATICBACKEND_HPP
