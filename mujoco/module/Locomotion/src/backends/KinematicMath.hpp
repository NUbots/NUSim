#ifndef K1SIM_MODULE_LOCOMOTION_BACKENDS_KINEMATICMATH_HPP
#define K1SIM_MODULE_LOCOMOTION_BACKENDS_KINEMATICMATH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <mujoco/mujoco.h>

#include "shared/sim/ModelMap.hpp"

// Small free functions shared by the kinematic root-velocity servo used in
// three places: KinematicBackend (steady-state WALKING/SOCCER), GetUpScript
// (second-half base ramp) and the Kicking transient (balance hold). Keeping
// this math in one header avoids re-deriving/duplicating the tilt-correction
// formula per caller.

namespace k1sim::module::backends {

// Smooth 0->1 ease (zero derivative at both ends) -- used for every scripted
// blend in this module (Prepare blend, GetUp phases, LieDown, Kick envelope).
inline double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

inline double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

inline std::array<double, 3> lerp_vec3(const std::array<double, 3>& a,
                                        const std::array<double, 3>& b,
                                        double t) {
    return {lerp(a[0], b[0], t), lerp(a[1], b[1], t), lerp(a[2], b[2], t)};
}

inline std::array<double, 3> normalize3(std::array<double, 3> v) {
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-9) {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
    }
    return v;
}

// The body's +z axis expressed in world coordinates ("up" if standing).
inline std::array<double, 3> up_vector(const double quat[4]) {
    std::array<double, 3> z_axis{0.0, 0.0, 1.0};
    std::array<double, 3> up{};
    mju_rotVecQuat(up.data(), z_axis.data(), quat);
    return up;
}

inline std::array<double, 3> base_up_vector(const mjModel* /*m*/, const mjData* d, const ModelMap& map) {
    const double* q = &d->qpos[map.root_qpos_adr + 3];  // wxyz
    return up_vector(q);
}

// Angle (rad) between the base's up vector and world-up. 0 = perfectly
// upright, pi/2 = lying on its side, pi = upside down.
inline double base_tilt(const mjModel* m, const mjData* d, const ModelMap& map) {
    const auto up = base_up_vector(m, d, map);
    return std::acos(std::clamp(up[2], -1.0, 1.0));
}

// Yaw (rotation about world z) of the base, extracted from the free joint's
// wxyz quaternion.
inline double base_yaw(const mjModel* /*m*/, const mjData* d, const ModelMap& map) {
    const double* q = &d->qpos[map.root_qpos_adr + 3];  // w,x,y,z
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

// Writes the 6 root free-joint qvel entries (see KinematicBackend / the plan's
// "Kinematic backend" section):
//   linear xy  = (vx_world, vy_world) -- already yaw-rotated by the caller
//   linear z   = k * (target_z - z)
//   angular xy = k * cross(up, target_up)   (drives "up" toward "target_up")
//   angular z  = vyaw (commanded yaw rate, independent of the tilt servo)
//
// IMPORTANT: MuJoCo's free-joint qvel convention is asymmetric (verified
// empirically -- see the workstream report) -- qvel[0:3] (linear) is in the
// WORLD frame, but qvel[3:6] (angular) is in the BODY frame. The tilt
// correction and vyaw above are naturally derived as a WORLD-frame angular
// velocity (that's the frame the "up -> target_up" cross product and the
// commanded yaw rate both make sense in), so it must be rotated into the
// body frame before being written to qvel -- otherwise, at any nonzero tilt,
// the correction fights itself and a commanded vyaw integrates yaw at the
// wrong rate (this was caught by the workstream's own numeric validation of
// the WALKING vyaw test, not by inspection).
inline void servo_root(const mjModel* /*m*/,
                        mjData* d,
                        const ModelMap& map,
                        double vx_world,
                        double vy_world,
                        double target_z,
                        const std::array<double, 3>& target_up,
                        double vyaw,
                        double k) {
    double* qvel        = &d->qvel[map.root_dof_adr];
    const double z       = d->qpos[map.root_qpos_adr + 2];
    const double* quat   = &d->qpos[map.root_qpos_adr + 3];
    const auto up        = base_up_vector(nullptr, d, map);

    // cross(up, target_up); this is the desired angular velocity's xy part,
    // expressed in WORLD frame (z is the commanded yaw rate, also world).
    const double ex = up[1] * target_up[2] - up[2] * target_up[1];
    const double ey = up[2] * target_up[0] - up[0] * target_up[2];
    const std::array<double, 3> omega_world{k * ex, k * ey, vyaw};

    double inv_quat[4];
    mju_negQuat(inv_quat, quat);
    std::array<double, 3> omega_body{};
    mju_rotVecQuat(omega_body.data(), omega_world.data(), inv_quat);

    qvel[0] = vx_world;
    qvel[1] = vy_world;
    qvel[2] = k * (target_z - z);
    qvel[3] = omega_body[0];
    qvel[4] = omega_body[1];
    qvel[5] = omega_body[2];
}

// Rotates a body-frame planar velocity (vx, vy) into world frame by the
// base's current yaw -- the Move command's semantics (SDK Move is body-frame).
inline std::array<double, 2> yaw_rotate(double vx, double vy, double yaw) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * vx - s * vy, s * vx + c * vy};
}

}  // namespace k1sim::module::backends

#endif  // K1SIM_MODULE_LOCOMOTION_BACKENDS_KINEMATICMATH_HPP
