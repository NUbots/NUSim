// quat_yaw (shared/sim/Quat.hpp), the heading shared by odometry, the head footprint pose and
// imu_state.rpy.
//
// Asserts:
//   - a pure rotation about z returns its angle, through +-90 degrees and 180 degrees.
//   - pitch and roll on top of a yaw (ZYX order, as the robot's body tilts) leave the yaw unchanged.
//   - the qpos-pointer and std::array overloads agree.
#include <array>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "shared/sim/Quat.hpp"

namespace {

    constexpr double kDeg = std::numbers::pi / 180.0;
    constexpr double kTol = 1e-12;

    int failures = 0;

    void check(bool cond, const char* what, double got, double want) {
        std::printf("[%s] %s: got %.6f, want %.6f\n", cond ? "PASS" : "FAIL", what, got, want);
        failures += cond ? 0 : 1;
    }

    // Angle difference wrapped to [-pi, pi], so 180 degrees matches -180 degrees
    double angle_diff(double a, double b) {
        return std::remainder(a - b, 2.0 * std::numbers::pi);
    }

    std::array<double, 4> axis_angle(double ax, double ay, double az, double angle) {
        const mjtNum axis[3]{ax, ay, az};
        std::array<double, 4> q{};
        mju_axisAngle2Quat(q.data(), axis, angle);
        return q;
    }

    // ZYX: yaw about world z, then pitch about the new y, then roll about the new x
    std::array<double, 4> from_ypr(double yaw, double pitch, double roll) {
        const auto qz = axis_angle(0, 0, 1, yaw);
        const auto qy = axis_angle(0, 1, 0, pitch);
        const auto qx = axis_angle(1, 0, 0, roll);
        std::array<double, 4> qzy{}, q{};
        mju_mulQuat(qzy.data(), qz.data(), qy.data());
        mju_mulQuat(q.data(), qzy.data(), qx.data());
        return q;
    }

}  // namespace

int main() {
    for (const double deg : {0.0, 30.0, 90.0, -90.0, 135.0, 180.0, -179.0}) {
        const double yaw = k1sim::quat_yaw(axis_angle(0, 0, 1, deg * kDeg));
        char what[64];
        std::snprintf(what, sizeof(what), "pure yaw %.0f deg", deg);
        check(std::abs(angle_diff(yaw, deg * kDeg)) < kTol, what, yaw / kDeg, deg);
    }

    // A fallen or leaning robot: large pitch/roll must not leak into the heading
    const double yaw_tilted = k1sim::quat_yaw(from_ypr(40.0 * kDeg, 60.0 * kDeg, -25.0 * kDeg));
    check(std::abs(angle_diff(yaw_tilted, 40.0 * kDeg)) < kTol,
          "yaw 40 under pitch 60 roll -25",
          yaw_tilted / kDeg,
          40.0);

    // Read the way a free joint's orientation is, from a qpos array: [x y z | w x y z]
    const auto q = from_ypr(-120.0 * kDeg, 10.0 * kDeg, 5.0 * kDeg);
    const mjtNum qpos[7]{1.0, 2.0, 0.5, q[0], q[1], q[2], q[3]};
    const double from_ptr = k1sim::quat_yaw(qpos + 3);
    const double from_arr = k1sim::quat_yaw(q);
    check(from_ptr == from_arr, "qpos pointer and array overloads agree", from_ptr / kDeg, from_arr / kDeg);

    return failures == 0 ? 0 : 1;
}
