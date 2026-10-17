#ifndef K1SIM_MODULE_SDKBRIDGE_ODOMETRYMODEL_HPP
#define K1SIM_MODULE_SDKBRIDGE_ODOMETRYMODEL_HPP

#include <array>
#include <cstdint>
#include <random>
#include <yaml-cpp/yaml.h>

// The robot's own odometry, as rt/odometer_state and rt/odom report it. The sim's base state is
// ground truth, but the real controller's odometry is an estimate that drifts; this corrupts the
// planar body velocity (forward, sideways, yaw rate) with the usual odometry error terms and
// integrates the pose from it, so the pose and the twist stay consistent with each other.
//
// Per axis: v_est = scale * v_true + bias + white noise, where the bias is a first-order
// Gauss-Markov process: stationary, with standard deviation bias_sigma, decorrelating over
// bias_time_constant. The white noise is a density, so the pose error it causes grows the same way
// whatever rate the state is published at: a white noise density s gives a position (or heading)
// random walk of s * sqrt(t). Over a window of T seconds the pose error variance is
//
//   s^2 T + 2 bias_sigma^2 tau^2 (T / tau - 1 + exp(-T / tau)),
//
// the bias drifting the pose steadily (like bias_sigma * T) over windows shorter than tau and
// adding to the random walk over longer ones.

namespace k1sim::module::sdkbridge {

    class OdometryModel {
    public:
        struct Config {
            bool enabled = false;  // false: publish the ground truth unchanged
            // Multiplicative error on the true body velocity (vx, vy, wz); 1 is exact
            std::array<double, 3> scale{1.0, 1.0, 1.0};
            // White noise density on the body velocity [m/sqrt(s), m/sqrt(s), rad/sqrt(s)]
            std::array<double, 3> velocity_noise_density{0.0, 0.0, 0.0};
            // Standard deviation of the velocity bias [m/s, m/s, rad/s]
            std::array<double, 3> bias_sigma{0.0, 0.0, 0.0};
            // Time over which the bias decorrelates [s]
            std::array<double, 3> bias_time_constant{10.0, 10.0, 10.0};
            uint64_t seed = 0;  // 0 seeds from std::random_device
        };

        // Planar pose in the odometry world frame, and the body-frame velocity that moved it
        struct Estimate {
            double x = 0.0, y = 0.0, yaw = 0.0;
            std::array<double, 3> velocity{};           // body frame (vx, vy, wz)
            std::array<double, 3> velocity_variance{};  // of this sample's velocity error
        };

        static Config load_config(const YAML::Node& node);

        explicit OdometryModel(const Config& config);

        // Advance to time t from the base's true planar pose and world-frame planar velocity. The
        // first call, and any call that goes back in time (a sim reset), starts again from the truth,
        // with a fresh bias drawn from its stationary distribution.
        const Estimate& update(double t, double x, double y, double yaw, double vx_world, double vy_world, double wz);

    private:
        Config config_;
        std::mt19937_64 rng_;
        std::normal_distribution<double> normal_{0.0, 1.0};

        bool initialised_ = false;
        double t_         = 0.0;
        std::array<double, 3> bias_{};
        Estimate estimate_;
    };

}  // namespace k1sim::module::sdkbridge

#endif  // K1SIM_MODULE_SDKBRIDGE_ODOMETRYMODEL_HPP
