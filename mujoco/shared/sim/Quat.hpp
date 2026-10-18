#ifndef K1SIM_SHARED_SIM_QUAT_HPP
#define K1SIM_SHARED_SIM_QUAT_HPP

#include <array>
#include <cmath>
#include <mujoco/mujoco.h>

namespace k1sim {

    // Yaw (heading about world z) of a {w, x, y, z} quaternion, in [-pi, pi]: the yaw of the ZYX
    // (yaw-pitch-roll) decomposition, which is the heading odometry and the base footprint use.
    // Takes a pointer so a free joint's orientation can be read straight out of qpos (qpos + 3).
    inline double quat_yaw(const mjtNum q[4]) {
        return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }

    inline double quat_yaw(const std::array<double, 4>& q) {
        return quat_yaw(q.data());
    }

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_QUAT_HPP
