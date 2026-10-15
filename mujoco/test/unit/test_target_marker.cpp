// rt/nusim/target_marker: the viewer fixes a robot-frame target in the world with robot_to_world, taking the base's
// heading from its quaternion with yaw_of.
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "shared/sim/PlanarPose.hpp"

namespace {

    bool g_ok = true;

    void fail(const std::string& what) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        g_ok = false;
    }

    bool approx(double a, double b, double eps = 1e-9) {
        return std::fabs(a - b) <= eps;
    }

    std::array<double, 4> yaw_quat(double yaw) {
        return {std::cos(yaw / 2), 0.0, 0.0, std::sin(yaw / 2)};
    }

    void test_yaw_of() {
        for (const double yaw : {0.0, 0.7, -2.5, M_PI / 2}) {
            if (!approx(k1sim::yaw_of(yaw_quat(yaw)), yaw)) {
                fail("yaw_of of a pure yaw quaternion " + std::to_string(yaw));
            }
        }
        // A pitched base keeps its heading: yaw 0.4 then pitch 0.3 (intrinsic z, then y)
        const double cy = std::cos(0.2), sy = std::sin(0.2), cp = std::cos(0.15), sp = std::sin(0.15);
        const std::array<double, 4> q{cy * cp, -sy * sp, cy * sp, sy * cp};
        if (!approx(k1sim::yaw_of(q), 0.4)) {
            fail("yaw_of of a pitched base");
        }
    }

    void test_robot_to_world() {
        // A target 1 m ahead and 0.5 m left, facing right, of a robot at (2, -1) facing +y
        const k1sim::PlanarPose robot{2.0, -1.0, M_PI / 2};
        const auto world = k1sim::robot_to_world({1.0, 0.5, -M_PI / 2}, robot);
        if (!approx(world.x, 1.5) || !approx(world.y, 0.0) || !approx(world.yaw, 0.0)) {
            fail("robot_to_world: robot facing +y");
        }
        // At the robot's own pose
        const auto here = k1sim::robot_to_world({}, robot);
        if (!approx(here.x, robot.x) || !approx(here.y, robot.y) || !approx(here.yaw, robot.yaw)) {
            fail("robot_to_world: the origin is the robot");
        }
        // Headings wrap into [-π, π]
        const auto wrapped = k1sim::robot_to_world({0.0, 0.0, 3.0}, {0.0, 0.0, 3.0});
        if (!approx(wrapped.yaw, 6.0 - 2.0 * M_PI)) {
            fail("robot_to_world: heading wraps");
        }
    }

}  // namespace

int main() {
    test_yaw_of();
    test_robot_to_world();
    if (g_ok) {
        std::printf("test_target_marker: OK\n");
    }
    return g_ok ? 0 : 1;
}
