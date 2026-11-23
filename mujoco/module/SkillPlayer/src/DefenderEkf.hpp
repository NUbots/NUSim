#ifndef K1SIM_MODULE_SKILLPLAYER_DEFENDEREKF_HPP
#define K1SIM_MODULE_SKILLPLAYER_DEFENDEREKF_HPP

#include <algorithm>
#include <array>
#include <cmath>

// The rondo_ekf drill's shared track of the defender: one extended Kalman filter that every
// teammate's camera detections of the robot in the middle are fused into. No sim, no NUClear, so it
// can be tested on its own (test/unit/test_defender_ekf.cpp).
//
// State: the defender's ground position and velocity [x, y, vx, vy] (field frame, m and m/s), with
// a constant-velocity model driven by white acceleration noise and a velocity that decays towards
// standing (a robot doesn't walk in a straight line for ever, so an unseen one shouldn't either).
//
// Measurement: one detection from one teammate's head camera, as a range and a bearing (field frame)
// from that camera's ground position: range from the size of the robot in the image (poor, and
// worse further off), bearing from where it is in the image (good). The bearing makes the update
// nonlinear in x, y: hence the EKF.
namespace k1sim::skill {

    struct EkfConfig {
        double accel_sigma      = 2.5;   // m/s^2, the white acceleration noise driving the velocity
        double velocity_tau     = 1.5;   // s: an unobserved velocity decays to 1/e in this long
        double init_speed_sigma = 1.0;   // m/s, the velocity's spread on a new track
        double gate             = 13.8;  // squared Mahalanobis distance a detection must be within (chi^2, 2 dof, 99.9%)
        int reinit_after        = 8;     // consecutive gated-out detections before the track starts again from one
    };

    // A detection by one camera: where the camera was (field x, y) and what it measured, with the
    // standard deviations it is believed to have.
    struct Detection {
        double cam_x = 0.0, cam_y = 0.0;
        double range = 0.0, bearing = 0.0;
        double sigma_range = 0.3, sigma_bearing = 0.03;
    };

    inline double ekf_wrap(double a) {
        constexpr double PI = 3.14159265358979323846;
        while (a > PI) {
            a -= 2.0 * PI;
        }
        while (a < -PI) {
            a += 2.0 * PI;
        }
        return a;
    }

    class DefenderEkf {
    public:
        using Vec4 = std::array<double, 4>;
        using Mat4 = std::array<std::array<double, 4>, 4>;

        DefenderEkf() = default;
        explicit DefenderEkf(const EkfConfig& config) : config_(config) {}

        void reset() {
            valid_    = false;
            rejected_ = 0;
            x_        = {};
            P_        = {};
        }

        bool valid() const {
            return valid_;
        }
        const Vec4& state() const {
            return x_;
        }
        const Mat4& covariance() const {
            return P_;
        }
        const EkfConfig& config() const {
            return config_;
        }

        // The 1-sigma length of the position ellipse's long axis (m); huge when there is no track.
        double position_sigma() const {
            if (!valid_) {
                return 1e9;
            }
            const double a = P_[0][0], b = P_[0][1], c = P_[1][1];
            const double mean = 0.5 * (a + c), diff = std::sqrt(0.25 * (a - c) * (a - c) + b * b);
            return std::sqrt(std::max(mean + diff, 0.0));
        }

        // Moves the track on by dt seconds.
        void predict(double dt) {
            if (!valid_ || dt <= 0.0) {
                return;
            }
            const double e = std::exp(-dt / std::max(config_.velocity_tau, 1e-3));
            Mat4 F{};
            F[0] = {1.0, 0.0, dt, 0.0};
            F[1] = {0.0, 1.0, 0.0, dt};
            F[2] = {0.0, 0.0, e, 0.0};
            F[3] = {0.0, 0.0, 0.0, e};
            x_ = {x_[0] + dt * x_[2], x_[1] + dt * x_[3], e * x_[2], e * x_[3]};
            // P = F P F^T + Q, with Q from white acceleration noise of accel_sigma.
            Mat4 FP{};
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    for (int k = 0; k < 4; ++k) {
                        FP[i][j] += F[i][k] * P_[k][j];
                    }
                }
            }
            Mat4 next{};
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    for (int k = 0; k < 4; ++k) {
                        next[i][j] += FP[i][k] * F[j][k];
                    }
                }
            }
            const double q   = config_.accel_sigma * config_.accel_sigma;
            const double pp  = q * dt * dt * dt * dt / 4.0, pv = q * dt * dt * dt / 2.0, vv = q * dt * dt;
            next[0][0] += pp;
            next[1][1] += pp;
            next[0][2] += pv;
            next[2][0] += pv;
            next[1][3] += pv;
            next[3][1] += pv;
            next[2][2] += vv;
            next[3][3] += vv;
            P_ = next;
        }

        // Fuses one detection. Returns false if it was gated out as inconsistent with the track (a
        // new track is started from it once reinit_after in a row have been).
        bool update(const Detection& z) {
            const double R00 = z.sigma_range * z.sigma_range, R11 = z.sigma_bearing * z.sigma_bearing;
            if (!valid_) {
                start(z, R00, R11);
                return true;
            }
            const double dx = x_[0] - z.cam_x, dy = x_[1] - z.cam_y;
            const double r2 = std::max(dx * dx + dy * dy, 1e-4), r = std::sqrt(r2);
            // H: d(range, bearing)/d(x, y, vx, vy); only the position columns are non-zero.
            const double H[2][2] = {{dx / r, dy / r}, {-dy / r2, dx / r2}};
            const double y[2]    = {z.range - r, ekf_wrap(z.bearing - std::atan2(dy, dx))};
            // PHt = P H^T (4x2); S = H P H^T + R (2x2).
            double PHt[4][2]{};
            for (int i = 0; i < 4; ++i) {
                for (int m = 0; m < 2; ++m) {
                    PHt[i][m] = P_[i][0] * H[m][0] + P_[i][1] * H[m][1];
                }
            }
            double S[2][2];
            for (int m = 0; m < 2; ++m) {
                for (int n = 0; n < 2; ++n) {
                    S[m][n] = H[m][0] * PHt[0][n] + H[m][1] * PHt[1][n];
                }
            }
            S[0][0] += R00;
            S[1][1] += R11;
            const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
            if (!(det > 1e-12)) {
                return false;
            }
            const double Si[2][2] = {{S[1][1] / det, -S[0][1] / det}, {-S[1][0] / det, S[0][0] / det}};
            const double d2 = y[0] * (Si[0][0] * y[0] + Si[0][1] * y[1]) + y[1] * (Si[1][0] * y[0] + Si[1][1] * y[1]);
            if (d2 > config_.gate) {
                if (++rejected_ >= config_.reinit_after) {
                    start(z, R00, R11);
                    return true;
                }
                return false;
            }
            rejected_ = 0;
            // K = P H^T S^-1 (4x2).
            double K[4][2];
            for (int i = 0; i < 4; ++i) {
                for (int n = 0; n < 2; ++n) {
                    K[i][n] = PHt[i][0] * Si[0][n] + PHt[i][1] * Si[1][n];
                }
            }
            for (int i = 0; i < 4; ++i) {
                x_[i] += K[i][0] * y[0] + K[i][1] * y[1];
            }
            // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T, which stays symmetric and positive.
            Mat4 A{};  // I - K H, with H padded to 2x4
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    const double kh = j < 2 ? K[i][0] * H[0][j] + K[i][1] * H[1][j] : 0.0;
                    A[i][j]         = (i == j ? 1.0 : 0.0) - kh;
                }
            }
            Mat4 AP{}, next{};
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    for (int k = 0; k < 4; ++k) {
                        AP[i][j] += A[i][k] * P_[k][j];
                    }
                }
            }
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    for (int k = 0; k < 4; ++k) {
                        next[i][j] += AP[i][k] * A[j][k];
                    }
                    next[i][j] += K[i][0] * R00 * K[j][0] + K[i][1] * R11 * K[j][1];
                }
            }
            P_ = next;
            return true;
        }

    private:
        // A new track at the detection, its position spread being the measurement's carried through
        // the polar-to-Cartesian Jacobian, and the velocity unknown.
        void start(const Detection& z, double R00, double R11) {
            const double c = std::cos(z.bearing), s = std::sin(z.bearing);
            x_ = {z.cam_x + z.range * c, z.cam_y + z.range * s, 0.0, 0.0};
            const double J[2][2] = {{c, -z.range * s}, {s, z.range * c}};
            P_                   = {};
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    P_[i][j] = J[i][0] * R00 * J[j][0] + J[i][1] * R11 * J[j][1];
                }
            }
            const double v = config_.init_speed_sigma * config_.init_speed_sigma;
            P_[2][2] = P_[3][3] = v;
            valid_              = true;
            rejected_           = 0;
        }

        EkfConfig config_{};
        bool valid_   = false;
        int rejected_ = 0;
        Vec4 x_{};
        Mat4 P_{};
    };

}  // namespace k1sim::skill

#endif  // K1SIM_MODULE_SKILLPLAYER_DEFENDEREKF_HPP
