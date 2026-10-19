#ifndef K1SIM_MODULE_SUPERVISOR_SUPERVISORPLACEMENT_HPP
#define K1SIM_MODULE_SUPERVISOR_SUPERVISORPLACEMENT_HPP

#include <array>
#include <cmath>
#include <mujoco/mujoco.h>

// MuJoCo qpos/qvel writers that teleport free-jointed bodies (the ball, a robot's root) for the
// GameController supervisor. Nothing here locks: callers must hold the sim mutex
// (message::SimHandles::mutex), which also keeps these testable without NUClear.
namespace k1sim::module::supervisor {

    // Yaw-only (about world +Z) wxyz quaternion; every placement pose here is a flat-ground standing
    // pose.
    inline std::array<double, 4> yaw_to_quat(double yaw_rad) {
        const double half = 0.5 * yaw_rad;
        return {std::cos(half), 0.0, 0.0, std::sin(half)};
    }

    // Writes body_id's free-joint qpos to the world position + yaw and zeros its qvel, dropping it in
    // at rest. Returns false without writing if body_id is invalid or doesn't own exactly one free
    // joint, so callers can log a config error instead of corrupting unrelated qpos.
    inline bool
        place_free_body(const mjModel* m, mjData* d, int body_id, double x, double y, double z, double yaw_rad) {
        if (m == nullptr || d == nullptr || body_id < 0 || body_id >= m->nbody) {
            return false;
        }
        if (m->body_jntnum[body_id] != 1) {
            return false;
        }
        const int jnt = m->body_jntadr[body_id];
        if (m->jnt_type[jnt] != mjJNT_FREE) {
            return false;
        }

        const int qpos_adr = m->jnt_qposadr[jnt];
        const int dof_adr  = m->jnt_dofadr[jnt];
        const auto q       = yaw_to_quat(yaw_rad);

        d->qpos[qpos_adr + 0] = x;
        d->qpos[qpos_adr + 1] = y;
        d->qpos[qpos_adr + 2] = z;
        d->qpos[qpos_adr + 3] = q[0];
        d->qpos[qpos_adr + 4] = q[1];
        d->qpos[qpos_adr + 5] = q[2];
        d->qpos[qpos_adr + 6] = q[3];
        for (int i = 0; i < 6; ++i) {
            d->qvel[dof_adr + i] = 0.0;
        }
        return true;
    }

    // Like place_free_body, but (x, y, z) is the world centre of one of the body's geoms rather than
    // the body origin; needed for the scene's "ball" body, whose sphere is offset from the body
    // origin (see k1_scene_robocup.xml). Places the body at identity orientation and assumes the
    // geom isn't rotated relative to it.
    inline bool place_free_body_by_geom_center(const mjModel* m,
                                               mjData* d,
                                               int body_id,
                                               int geom_id,
                                               double x,
                                               double y,
                                               double z) {
        if (m == nullptr || geom_id < 0 || geom_id >= m->ngeom || m->geom_bodyid[geom_id] != body_id) {
            return false;
        }
        const double ox = m->geom_pos[3 * geom_id + 0];
        const double oy = m->geom_pos[3 * geom_id + 1];
        const double oz = m->geom_pos[3 * geom_id + 2];
        return place_free_body(m, d, body_id, x - ox, y - oy, z - oz, 0.0);
    }

}  // namespace k1sim::module::supervisor

#endif  // K1SIM_MODULE_SUPERVISOR_SUPERVISORPLACEMENT_HPP
