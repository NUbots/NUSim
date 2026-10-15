#ifndef K1SIM_SHARED_SIM_PLANARPOSE_HPP
#define K1SIM_SHARED_SIM_PLANARPOSE_HPP

#include <array>
#include <cmath>

namespace k1sim {

    // A pose on the floor: position (m) and heading (rad, about world z)
    struct PlanarPose {
        double x   = 0.0;
        double y   = 0.0;
        double yaw = 0.0;
    };

    // The yaw of a w,x,y,z quaternion (rotation about z of its heading)
    inline double yaw_of(const std::array<double, 4>& q) {
        return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }

    // A pose given in the robot's yaw-only frame (x forward, at the base's ground projection) into the world, for the
    // robot's base at `robot`
    inline PlanarPose robot_to_world(const PlanarPose& pose_in_robot, const PlanarPose& robot) {
        const double c = std::cos(robot.yaw);
        const double s = std::sin(robot.yaw);
        return {robot.x + c * pose_in_robot.x - s * pose_in_robot.y,
                robot.y + s * pose_in_robot.x + c * pose_in_robot.y,
                std::remainder(robot.yaw + pose_in_robot.yaw, 2.0 * M_PI)};
    }

}  // namespace k1sim

#endif  // K1SIM_SHARED_SIM_PLANARPOSE_HPP
