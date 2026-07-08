// M2 acceptance: SimCore's PD-to-ready fallback (no StepController attached) holds the K1
// standing for 60s of sim time in free-run (rtf=0).
//
// Drives the real SimCore physics thread (not a hand-rolled loop) so this also exercises the
// free-run pacing path and reports its stepping rate. state_publish_divisor is set to 1 so the
// SimStateUpdate callback (fired from the physics thread — see SimCore::physics_loop) gives
// deterministic per-step coverage instead of sampling from a second thread.
//
// Model path resolution mirrors test_model_load.cpp: $K1SIM_TEST_MODEL overrides
// config/simulation.yaml's `model` key.
//
// Asserts:
//   - no NaN appears in qpos (base pose/orientation + all 22 joint angles) at any point.
//   - base height stays within [0.45, 0.62] for the whole run once t > 1s.
//   - tilt (angle between the base's local z-axis and world z) is < 10 deg at the end.
//   - the free-run stepping rate is sane (> 1000 steps/s).
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "module/Simulation/src/SimCore.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string resolve_test_model_path() {
    if (const char* override_path = std::getenv("K1SIM_TEST_MODEL")) {
        return override_path;
    }
    auto cfg = k1sim::config::load("simulation.yaml");
    return k1sim::config::resolve_path(cfg["model"].as<std::string>()).string();
}

double clamp(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

int main() {
    using k1sim::JOINT_COUNT;
    using k1sim::message::SimStateUpdate;

    k1sim::SimCore::Config cfg;
    cfg.model_path = resolve_test_model_path();
    cfg.rtf        = 0.0;  // free-run: no pacing sleep
    cfg.state_publish_divisor = 1;
    cfg.resync_threshold      = 0.05;  // unused in free-run

    // Gains + fallback pose come from config/gains.yaml, same as production — this test is
    // exactly what gates any tuning of that file.
    auto gains_cfg = k1sim::config::load("gains.yaml");
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        cfg.kp[i]                  = gains_cfg["kp"][i].as<double>();
        cfg.kd[i]                  = gains_cfg["kd"][i].as<double>();
        cfg.ready_pose_fallback[i] = gains_cfg["ready_pose"][i].as<double>();
    }

    std::atomic<bool> saw_nan{false};
    std::atomic<bool> height_violation{false};
    std::atomic<double> last_tilt_deg{0.0};
    std::atomic<double> last_sim_time{0.0};
    std::atomic<double> min_height_after_1s{1e9};
    std::atomic<double> max_height_after_1s{-1e9};
    std::atomic<bool> done{false};

    auto on_state = [&](std::unique_ptr<SimStateUpdate> s) {
        last_sim_time.store(s->sim_time, std::memory_order_relaxed);

        bool nan_here = std::isnan(s->base.x) || std::isnan(s->base.y) || std::isnan(s->base.z);
        for (double v : s->base.quat) {
            nan_here = nan_here || std::isnan(v);
        }
        for (const auto& j : s->joints) {
            nan_here = nan_here || std::isnan(j.q);
        }
        if (nan_here) {
            saw_nan.store(true, std::memory_order_relaxed);
        }

        if (s->sim_time > 1.0) {
            if (s->base.z < 0.45 || s->base.z > 0.62) {
                height_violation.store(true, std::memory_order_relaxed);
            }
            double cur_min = min_height_after_1s.load(std::memory_order_relaxed);
            if (s->base.z < cur_min) {
                min_height_after_1s.store(s->base.z, std::memory_order_relaxed);
            }
            double cur_max = max_height_after_1s.load(std::memory_order_relaxed);
            if (s->base.z > cur_max) {
                max_height_after_1s.store(s->base.z, std::memory_order_relaxed);
            }
        }

        // tilt = angle between the base's local z-axis and world z, from the wxyz quat.
        const double qw = s->base.quat[0];
        const double qx = s->base.quat[1];
        const double qy = s->base.quat[2];
        const double zz = 1.0 - 2.0 * (qx * qx + qy * qy);
        (void) qw;
        const double tilt_rad = std::acos(clamp(zz, -1.0, 1.0));
        last_tilt_deg.store(tilt_rad * 180.0 / kPi, std::memory_order_relaxed);

        if (s->sim_time >= 60.0) {
            done.store(true, std::memory_order_relaxed);
        }
    };

    k1sim::SimCore sim(cfg, on_state);

    try {
        sim.load_model();
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "load_model failed: %s\n", e.what());
        return 1;
    }

    const auto wall_start = std::chrono::steady_clock::now();
    const auto wall_deadline = wall_start + std::chrono::seconds(60);  // generous CI safety net
    sim.start();

    while (!done.load(std::memory_order_relaxed)) {
        if (std::chrono::steady_clock::now() > wall_deadline) {
            std::fprintf(stderr, "test_pd_stand: timed out waiting for 60s of sim time\n");
            sim.stop();
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto wall_end = std::chrono::steady_clock::now();

    sim.stop();

    const double wall_elapsed = std::chrono::duration<double>(wall_end - wall_start).count();
    const uint64_t steps      = sim.step_count();
    const double steps_per_sec = wall_elapsed > 0.0 ? static_cast<double>(steps) / wall_elapsed : 0.0;

    bool ok = true;
    if (saw_nan.load()) {
        std::fprintf(stderr, "FAIL: NaN detected in qpos during the run\n");
        ok = false;
    }
    if (height_violation.load()) {
        std::fprintf(stderr,
                      "FAIL: base height left [0.45, 0.62] after t=1s (min=%.4f max=%.4f)\n",
                      min_height_after_1s.load(),
                      max_height_after_1s.load());
        ok = false;
    }
    if (last_tilt_deg.load() >= 10.0) {
        std::fprintf(stderr, "FAIL: final tilt %.2f deg >= 10 deg\n", last_tilt_deg.load());
        ok = false;
    }
    if (steps_per_sec <= 1000.0) {
        std::fprintf(stderr, "FAIL: free-run stepping rate %.1f steps/s <= 1000\n", steps_per_sec);
        ok = false;
    }

    std::printf(
        "test_pd_stand: sim_time=%.3fs steps=%llu wall=%.3fs (%.0f steps/s) "
        "height[1s..end]=[%.4f, %.4f] final_tilt=%.2fdeg\n",
        last_sim_time.load(),
        static_cast<unsigned long long>(steps),
        wall_elapsed,
        steps_per_sec,
        min_height_after_1s.load(),
        max_height_after_1s.load(),
        last_tilt_deg.load());

    return ok ? 0 : 1;
}
