// OdometryModel, the error model of rt/odometer_state and rt/odom (config/odometry.yaml).
//
// Asserts:
//   - disabled, it passes the ground truth through unchanged.
//   - a scale error scales the distance walked and the angle turned.
//   - white noise density s spreads the position after T seconds standing still with standard
//     deviation s * sqrt(T), and a Gauss-Markov bias (sigma b, time constant tau) with
//     sqrt(2 b^2 tau^2 (T / tau - 1 + exp(-T / tau))), each within 15% over 400 seeded runs.
//   - the same seed gives the same odometry, and going back in time (a sim reset) restarts from
//     the ground truth.
#include <cmath>
#include <cstdio>
#include <vector>

#include "module/SdkBridge/src/OdometryModel.hpp"

using k1sim::module::sdkbridge::OdometryModel;

namespace {

    constexpr double kDt = 0.02;  // the 50 Hz state publish rate

    int failures = 0;

    void check(bool cond, const char* what, double got, double want) {
        std::printf("[%s] %s: got %.4f, want %.4f\n", cond ? "PASS" : "FAIL", what, got, want);
        failures += cond ? 0 : 1;
    }

    // Walk at a constant body velocity (vx, wz) for `seconds`, returning the final estimate
    OdometryModel::Estimate walk(OdometryModel& model, double vx, double wz, double seconds) {
        double x = 0, y = 0, yaw = 0;
        OdometryModel::Estimate est = model.update(0.0, x, y, yaw, 0.0, 0.0, 0.0);
        for (int i = 1; i <= int(std::lround(seconds / kDt)); ++i) {
            const double vxw = vx * std::cos(yaw), vyw = vx * std::sin(yaw);
            x += vxw * kDt;
            y += vyw * kDt;
            yaw += wz * kDt;
            est = model.update(i * kDt, x, y, yaw, vxw, vyw, wz);
        }
        return est;
    }

    // Standard deviation of the final x after standing still for `seconds`, over `runs` seeds
    double spread(OdometryModel::Config cfg, double seconds, int runs) {
        double sum = 0, sum2 = 0;
        for (int r = 0; r < runs; ++r) {
            cfg.seed = 1000 + r;
            OdometryModel model(cfg);
            const double x = walk(model, 0.0, 0.0, seconds).x;
            sum += x;
            sum2 += x * x;
        }
        const double mean = sum / runs;
        return std::sqrt(sum2 / runs - mean * mean);
    }

}  // namespace

int main() {
    {
        OdometryModel::Config cfg;
        cfg.enabled                = false;
        cfg.velocity_noise_density = {0.5, 0.5, 0.5};
        OdometryModel model(cfg);
        model.update(0.0, 0, 0, 0, 0, 0, 0);
        const auto& est = model.update(0.02, 1.25, -0.5, 0.3, 0.4, 0.1, 0.2);
        check(est.x == 1.25 && est.y == -0.5 && est.yaw == 0.3, "disabled passes the pose through", est.x, 1.25);
    }
    {
        OdometryModel::Config cfg;
        cfg.enabled = true;
        cfg.scale   = {1.1, 1.0, 1.0};
        OdometryModel model(cfg);
        const auto est = walk(model, 0.3, 0.0, 10.0);
        check(std::abs(est.x - 3.3) < 1e-6, "forward scale 1.1 over 3 m walked", est.x, 3.3);
    }
    {
        OdometryModel::Config cfg;
        cfg.enabled = true;
        cfg.scale   = {1.0, 1.0, 0.9};
        OdometryModel model(cfg);
        const auto est = walk(model, 0.0, 0.5, 4.0);
        check(std::abs(est.yaw - 1.8) < 1e-6, "yaw scale 0.9 over 2 rad turned", est.yaw, 1.8);
    }
    {
        OdometryModel::Config cfg;
        cfg.enabled                = true;
        cfg.velocity_noise_density = {0.02, 0.0, 0.0};
        const double got           = spread(cfg, 10.0, 400);
        const double want          = 0.02 * std::sqrt(10.0);
        check(std::abs(got / want - 1.0) < 0.15, "white noise position spread after 10 s", got, want);
    }
    {
        OdometryModel::Config cfg;
        cfg.enabled            = true;
        cfg.bias_sigma         = {0.01, 0.0, 0.0};
        cfg.bias_time_constant = {5.0, 5.0, 5.0};
        const double T = 10.0, tau = 5.0, b = 0.01;
        const double got  = spread(cfg, T, 400);
        const double want = std::sqrt(2.0 * b * b * tau * tau * (T / tau - 1.0 + std::exp(-T / tau)));
        check(std::abs(got / want - 1.0) < 0.15, "Gauss-Markov bias position drift after 10 s", got, want);
    }
    {
        OdometryModel::Config cfg;
        cfg.enabled                = true;
        cfg.velocity_noise_density = {0.05, 0.05, 0.05};
        cfg.seed                   = 7;
        OdometryModel a(cfg), b(cfg);
        const double xa = walk(a, 0.3, 0.2, 5.0).x, xb = walk(b, 0.3, 0.2, 5.0).x;
        check(xa == xb, "same seed, same odometry", xa, xb);

        // A sim reset rewinds time: the estimate restarts from the ground truth
        const auto& est = a.update(0.0, 2.0, 1.0, 0.5, 0, 0, 0);
        check(est.x == 2.0 && est.y == 1.0 && est.yaw == 0.5, "a sim reset restarts from the truth", est.x, 2.0);
    }

    std::printf("test_odometry_model: %s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
