#include "module/SdkBridge/src/OdometryModel.hpp"

#include <cmath>

namespace k1sim::module::sdkbridge {

    namespace {

        // Avoid M_PI: not reliably available under this project's -std=c++17 build
        constexpr double kPi = 3.14159265358979323846;

        std::array<double, 3> triple(const YAML::Node& node, const std::array<double, 3>& fallback) {
            if (!node) {
                return fallback;
            }
            return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
        }

    }  // namespace

    OdometryModel::Config OdometryModel::load_config(const YAML::Node& node) {
        Config cfg;
        cfg.enabled                = node["enabled"].as<bool>(cfg.enabled);
        cfg.scale                  = triple(node["scale"], cfg.scale);
        cfg.velocity_noise_density = triple(node["velocity_noise_density"], cfg.velocity_noise_density);
        cfg.bias_sigma             = triple(node["bias_sigma"], cfg.bias_sigma);
        cfg.bias_time_constant     = triple(node["bias_time_constant"], cfg.bias_time_constant);
        cfg.seed                   = node["seed"].as<uint64_t>(cfg.seed);
        return cfg;
    }

    OdometryModel::OdometryModel(const Config& config)
        : config_(config), rng_(config.seed != 0 ? config.seed : std::random_device{}()) {}

    const OdometryModel::Estimate&
        OdometryModel::update(double t, double x, double y, double yaw, double vx_world, double vy_world, double wz) {
        // The true velocity in the yaw-only body frame
        const double c = std::cos(yaw), s = std::sin(yaw);
        const std::array<double, 3> v_true{c * vx_world + s * vy_world, -s * vx_world + c * vy_world, wz};

        const double dt = t - t_;
        if (!config_.enabled || !initialised_ || dt <= 0.0) {
            estimate_ = Estimate{x, y, yaw, v_true, {}};
            for (std::size_t i = 0; i < 3; ++i) {
                bias_[i] = config_.bias_sigma[i] * normal_(rng_);
            }
            initialised_ = true;
            t_           = t;
            return estimate_;
        }
        t_ = t;

        for (std::size_t i = 0; i < 3; ++i) {
            // Exact discretisation of the Gauss-Markov bias, stationary at bias_sigma
            const double a = std::exp(-dt / config_.bias_time_constant[i]);
            bias_[i]       = a * bias_[i] + config_.bias_sigma[i] * std::sqrt(1.0 - a * a) * normal_(rng_);
            // A white noise density sigma is a per-sample standard deviation of sigma / sqrt(dt)
            const double sample_sigma      = config_.velocity_noise_density[i] / std::sqrt(dt);
            estimate_.velocity[i]          = config_.scale[i] * v_true[i] + bias_[i] + sample_sigma * normal_(rng_);
            estimate_.velocity_variance[i] = sample_sigma * sample_sigma;
        }

        // Integrate the pose from the estimated velocity, rotating it by the midpoint heading
        const double yaw_mid = estimate_.yaw + 0.5 * estimate_.velocity[2] * dt;
        const double cm = std::cos(yaw_mid), sm = std::sin(yaw_mid);
        estimate_.x += (cm * estimate_.velocity[0] - sm * estimate_.velocity[1]) * dt;
        estimate_.y += (sm * estimate_.velocity[0] + cm * estimate_.velocity[1]) * dt;
        estimate_.yaw = std::remainder(estimate_.yaw + estimate_.velocity[2] * dt, 2.0 * kPi);
        return estimate_;
    }

}  // namespace k1sim::module::sdkbridge
