#ifndef K1SIM_SHARED_SIM_SHOVE_HPP
#define K1SIM_SHARED_SIM_SHOVE_HPP

#include <mujoco/mujoco.h>

namespace k1sim::sim {

// Shove the main robot over: a deterministic fall for testing FallRecovery/GetUp (mouse-drag
// perturbs are usually within what the push-randomised policy rides out). The caller must hold
// the sim mutex.
inline void shove_robot(const mjModel* m, mjData* d) {
    if (m->njnt > 0 && m->jnt_type[0] == mjJNT_FREE) {
        const int dof = m->jnt_dofadr[0];
        d->qvel[dof + 0] += 1.5;  // linear kick, world x
        d->qvel[dof + 4] += 6.0;  // pitch rate — guarantees a topple
    }
}

}  // namespace k1sim::sim

#endif  // K1SIM_SHARED_SIM_SHOVE_HPP
