#ifndef K1SIM_MODULE_LOCOMOTION_LOCOMATH_HPP
#define K1SIM_MODULE_LOCOMOTION_LOCOMATH_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <mujoco/mujoco.h>

#include "shared/sim/ModelMap.hpp"

// Small free functions for the Prepare blend and fall detection. (The larger
// kinematic root-servo math that used to live in backends/KinematicMath.hpp
// left with the locomotion backends -- locomotion policies now run on the
// NUbots_K1 side and reach the sim as LowCmd servo targets.)

namespace k1sim::module {

// Smooth 0->1 ease (zero derivative at both ends) -- the Prepare blend shape.
inline double smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

inline double lerp(double a, double b, double t) {
    return a + (b - a) * t;
}

// The body's +z axis expressed in world coordinates ("up" if standing).
inline std::array<double, 3> base_up_vector(const mjData* d, const ModelMap& map) {
    const double* quat = &d->qpos[map.root_qpos_adr + 3];  // wxyz
    std::array<double, 3> z_axis{0.0, 0.0, 1.0};
    std::array<double, 3> up{};
    mju_rotVecQuat(up.data(), z_axis.data(), quat);
    return up;
}

// Angle (rad) between the base's up vector and world-up. 0 = perfectly
// upright, pi/2 = lying on its side, pi = upside down.
inline double base_tilt(const mjData* d, const ModelMap& map) {
    const auto up = base_up_vector(d, map);
    return std::acos(std::clamp(up[2], -1.0, 1.0));
}

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_LOCOMATH_HPP
