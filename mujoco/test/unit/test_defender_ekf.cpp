// rondo_ekf's shared defender track (module/SkillPlayer/src/DefenderEkf.hpp), with no sim:
//   A. the first detection starts a track at the measured spot
//   B. three cameras' noisy detections of a standing robot converge on it, tighter than one camera alone
//   C. a walking robot is followed, velocity included
//   D. unseen, the track coasts and its doubt grows; the velocity fades
//   E. a wild detection is gated out; enough of them in a row start a new track there
#include <cmath>
#include <cstdio>
#include <random>

#include "module/SkillPlayer/src/DefenderEkf.hpp"

namespace {

    using namespace k1sim::skill;

    int failures = 0;

    void check(bool ok, const char* what) {
        if (!ok) {
            std::printf("FAIL: %s\n", what);
            ++failures;
        }
    }

    constexpr double DT = 0.02;

    // A detection of (x, y) from a camera at (cx, cy), with noise as the drill's defaults.
    Detection see(double cx, double cy, double x, double y, std::mt19937& rng, bool noisy = true) {
        std::normal_distribution<double> unit(0.0, 1.0);
        Detection z;
        z.cam_x         = cx;
        z.cam_y         = cy;
        const double r  = std::hypot(x - cx, y - cy);
        z.sigma_range   = 0.1 + 0.12 * r;
        z.sigma_bearing = 0.03;
        z.range         = r + (noisy ? z.sigma_range * unit(rng) : 0.0);
        z.bearing       = std::atan2(y - cy, x - cx) + (noisy ? z.sigma_bearing * unit(rng) : 0.0);
        return z;
    }

    const double CAMS[3][2] = {{0.0, -3.2}, {2.75, 1.6}, {-2.75, 1.6}};

}  // namespace

int main() {
    std::mt19937 rng(7);

    {  // A
        DefenderEkf ekf;
        check(!ekf.valid(), "A: no track before a detection");
        ekf.update(see(0.0, -3.0, 0.5, 0.0, rng, false));
        check(ekf.valid(), "A: a detection starts a track");
        check(std::hypot(ekf.state()[0] - 0.5, ekf.state()[1]) < 1e-9, "A: at the measured spot");
    }

    {  // B
        DefenderEkf one, all;
        for (int i = 0; i < 150; ++i) {
            one.predict(DT);
            all.predict(DT);
            one.update(see(CAMS[0][0], CAMS[0][1], 0.3, 0.2, rng));
            for (const auto& c : CAMS) {
                all.update(see(c[0], c[1], 0.3, 0.2, rng));
            }
        }
        const double error = std::hypot(all.state()[0] - 0.3, all.state()[1] - 0.2);
        check(error < 0.15, "B: three cameras put a standing robot within 15 cm");
        check(all.position_sigma() < one.position_sigma(), "B: three cameras are surer than one");
        // One camera's doubt is mostly along its line of sight (range is the poor measurement).
        const auto& P = one.covariance();
        check(P[1][1] > 3.0 * P[0][0], "B: one camera is surer across its line of sight than along it");
    }

    {  // C
        DefenderEkf ekf;
        double x = -1.0, y = -0.5;
        const double vx = 0.8, vy = 0.4;
        for (int i = 0; i < 200; ++i) {
            x += vx * DT;
            y += vy * DT;
            ekf.predict(DT);
            if (i % 3 == 0) {
                for (const auto& c : CAMS) {
                    ekf.update(see(c[0], c[1], x, y, rng));
                }
            }
        }
        check(std::hypot(ekf.state()[0] - x, ekf.state()[1] - y) < 0.25, "C: a walking robot is followed");
        check(std::hypot(ekf.state()[2] - vx, ekf.state()[3] - vy) < 0.35, "C: its velocity is estimated");
    }

    {  // D
        DefenderEkf ekf;
        for (int i = 0; i < 100; ++i) {
            ekf.predict(DT);
            for (const auto& c : CAMS) {
                ekf.update(see(c[0], c[1], 0.02 * i, 0.0, rng));  // walking at 1 m/s along x
            }
        }
        const double sigma = ekf.position_sigma(), speed = ekf.state()[2];
        for (int i = 0; i < 100; ++i) {
            ekf.predict(DT);
        }
        check(ekf.position_sigma() > 2.0 * sigma, "D: unseen, the doubt grows");
        check(ekf.state()[2] < 0.5 * speed && ekf.state()[2] > 0.0, "D: unseen, the velocity fades");
    }

    {  // E
        EkfConfig config;
        config.reinit_after = 5;
        DefenderEkf ekf(config);
        for (int i = 0; i < 50; ++i) {
            ekf.predict(DT);
            for (const auto& c : CAMS) {
                ekf.update(see(c[0], c[1], 0.0, 0.0, rng));
            }
        }
        check(!ekf.update(see(0.0, -3.2, 2.0, 2.0, rng, false)), "E: a wild detection is gated out");
        check(std::hypot(ekf.state()[0], ekf.state()[1]) < 0.2, "E: and doesn't move the track");
        for (int i = 0; i < 4; ++i) {
            ekf.update(see(0.0, -3.2, 2.0, 2.0, rng, false));
        }
        check(std::hypot(ekf.state()[0] - 2.0, ekf.state()[1] - 2.0) < 1e-6, "E: enough in a row start a new track there");
    }

    if (failures == 0) {
        std::printf("test_defender_ekf: all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
