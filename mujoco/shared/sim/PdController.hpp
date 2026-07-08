#ifndef K1SIM_SHARED_SIM_PDCONTROLLER_HPP
#define K1SIM_SHARED_SIM_PDCONTROLLER_HPP

#include <algorithm>
#include <array>

#include "shared/k1/JointIndex.hpp"
#include "shared/sim/ModelMap.hpp"

namespace k1sim {

// Per-joint PD over the 22 actuated joints (JointIndexK1 order). Gains come from
// config/gains.yaml. Torques are clamped to the model's actuator forcerange.
// Used by the Prepare stand, the GetUp script and both locomotion backends.
class PdController {
public:
    std::array<double, JOINT_COUNT> kp{};
    std::array<double, JOINT_COUNT> kd{};

    // ctrl[i] = clamp(kp*(q_ref - q) + kd*(dq_ref - dq) + tau_ff, forcerange)
    // Writes d->ctrl for all 22 actuators; q_ref/dq_ref/tau_ff in JointIndexK1 order.
    void apply(const mjModel* m,
               mjData* d,
               const ModelMap& map,
               const std::array<double, JOINT_COUNT>& q_ref,
               const std::array<double, JOINT_COUNT>& dq_ref = {},
               const std::array<double, JOINT_COUNT>& tau_ff = {}) const {
        for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
            const double q   = d->qpos[map.qpos_adr[i]];
            const double dq  = d->qvel[map.dof_adr[i]];
            double tau       = kp[i] * (q_ref[i] - q) + kd[i] * (dq_ref[i] - dq) + tau_ff[i];
            const int act    = map.act_id[i];
            const double lo  = m->actuator_forcerange[2 * act];
            const double hi  = m->actuator_forcerange[2 * act + 1];
            if (m->actuator_forcelimited[act] != 0) {
                tau = std::clamp(tau, lo, hi);
            }
            d->ctrl[act] = tau;
        }
    }
};

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_PDCONTROLLER_HPP
