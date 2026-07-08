// Contract-test client: uses the Booster SDK strictly AS A CLIENT (never linked
// into k1_mujoco_sim) to prove module::SdkBridge is byte-compatible with the real
// wire protocol. Built on the HOST (see build.sh) against the older-but-wire-
// identical local checkout at /home/nubots/Workspace/booster/booster_robotics_sdk
// (commit 7fb7287) — see module/SdkBridge/PROTOCOL.md for why that's equivalent to
// the pinned 324946e7 for every type/api_id this test touches.
//
// Checks (see module/SdkBridge/PROTOCOL.md and the M4/M5 acceptance criteria):
//   - rt/low_state received at >= 45 msgs/s over a 5s window
//   - each sample's motor_state_serial has exactly 22 entries with finite,
//     plausible-magnitude values
//   - rt/odometer_state received at least once
//   - rt/battery_state received at least once during the 5s window (published at
//     1 Hz via NUClear on<Every<1, std::chrono::seconds>>) with soc in (0, 100].
//     Only when the SDK being built against ships booster/idl/b1/BatteryState.h —
//     the pinned 324946e7 does, the older local checkout (7fb7287) doesn't even
//     compile the type into its .a; guarded with __has_include, SKIP printed
//     otherwise. Point BOOSTER_SDK_ROOT at a pinned-SDK extract to enable it.
//   - B1LocoClient::ChangeMode(kPrepare)/Move/RotateHead/GetUp each return 0
//     within 1000 ms (the SDK's own client-side timeout)
//   - B1LocoClient::GetMode returns 0 and reports kPrepare after the
//     ChangeMode(kPrepare) settles — exercises the RPC *response body* path
//     ({"mode": N}), not just the status echo
//
// Prints one PASS/FAIL line per check plus a summary; exit code 0 iff all passed.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>

// booster/idl/b1/BatteryState.h shipping is a pinned-SDK (324946e7) marker: that
// version also ships BatteryState_ in its .a AND requires ChannelFactory::
// InitDefault(domain) instead of Init(domain) (whose empty-interface overload
// tries to load a FASTRTPS_DEFAULT_PROFILES_FILE XML and fails without one).
// The older local checkout (7fb7287) has neither.
#if __has_include(<booster/idl/b1/BatteryState.h>)
    #include <booster/idl/b1/BatteryState.h>
    #define K1SIM_CHECK_BATTERY 1
    #define K1SIM_SDK_PINNED 1
#endif
#include <booster/idl/b1/LowState.h>
#include <booster/idl/b1/Odometer.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/b1/b1_loco_client.hpp>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>

using namespace booster::robot;
using namespace booster_interface::msg;

namespace {

std::atomic<uint64_t> g_low_state_count{0};
std::atomic<bool> g_low_state_plausible{true};
std::atomic<int> g_last_motor_count{-1};

void LowStateHandler(const void* msg) {
    const auto* state = static_cast<const LowState*>(msg);
    g_low_state_count.fetch_add(1, std::memory_order_relaxed);
    g_last_motor_count.store(static_cast<int>(state->motor_state_serial().size()), std::memory_order_relaxed);
    for (const auto& m : state->motor_state_serial()) {
        if (!std::isfinite(m.q()) || !std::isfinite(m.dq()) || !std::isfinite(m.tau_est())
            || std::fabs(m.q()) > 100.0f) {
            g_low_state_plausible.store(false, std::memory_order_relaxed);
        }
    }
    for (float v : state->imu_state().acc()) {
        if (!std::isfinite(v)) {
            g_low_state_plausible.store(false, std::memory_order_relaxed);
        }
    }
}

std::atomic<uint64_t> g_odom_count{0};
void OdometerHandler(const void* /*msg*/) {
    g_odom_count.fetch_add(1, std::memory_order_relaxed);
}

#ifdef K1SIM_CHECK_BATTERY
std::atomic<uint64_t> g_battery_count{0};
std::atomic<float> g_battery_soc{-1.0f};
void BatteryHandler(const void* msg) {
    const auto* battery = static_cast<const BatteryState*>(msg);
    g_battery_count.fetch_add(1, std::memory_order_relaxed);
    g_battery_soc.store(battery->soc(), std::memory_order_relaxed);
}
#endif

int g_failures = 0;

void check(bool cond, const std::string& what) {
    std::printf("[%s] %s\n", cond ? "PASS" : "FAIL", what.c_str());
    if (!cond) {
        ++g_failures;
    }
}

void timed_rpc(const char* name, const std::function<int32_t()>& fn) {
    const auto start = std::chrono::steady_clock::now();
    const int32_t ret = fn();
    const auto end     = std::chrono::steady_clock::now();
    const double ms    = std::chrono::duration<double, std::milli>(end - start).count();
    std::printf("  %s -> ret=%d, latency=%.2f ms\n", name, ret, ms);
    check(ret == 0, std::string(name) + " returned 0");
    check(ms < 1000.0, std::string(name) + " completed within 1000 ms");
}

}  // namespace

int main() {
    std::printf("=== k1sim SdkBridge contract test (host SDK client) ===\n");

#ifdef K1SIM_SDK_PINNED
    ChannelFactory::Instance()->InitDefault(0);
#else
    ChannelFactory::Instance()->Init(0);
#endif

    ChannelSubscriber<LowState> low_state_sub(booster::robot::b1::kTopicLowState, LowStateHandler);
    low_state_sub.InitChannel();
    ChannelSubscriber<Odometer> odom_sub(booster::robot::b1::kTopicOdometerState, OdometerHandler);
    odom_sub.InitChannel();
#ifdef K1SIM_CHECK_BATTERY
    ChannelSubscriber<BatteryState> battery_sub("rt/battery_state", BatteryHandler);
    battery_sub.InitChannel();
#endif

    // Let DDS discovery settle before measuring rate.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    g_low_state_count.store(0);
    g_odom_count.store(0);
#ifdef K1SIM_CHECK_BATTERY
    g_battery_count.store(0);
#endif
    const auto rate_start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    const double elapsed_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - rate_start).count();
    const double hz        = static_cast<double>(g_low_state_count.load()) / elapsed_s;

    std::printf("low_state: %.2f msgs/s over %.2fs (%llu samples total), last motor_state_serial size=%d\n",
                hz,
                elapsed_s,
                static_cast<unsigned long long>(g_low_state_count.load()),
                g_last_motor_count.load());
    check(hz >= 45.0, "rt/low_state received at >= 45 msgs/s");
    check(g_last_motor_count.load() == 22, "rt/low_state motor_state_serial has exactly 22 entries");
    check(g_low_state_plausible.load(), "rt/low_state motor/imu values are finite and plausible magnitude");
    check(g_odom_count.load() > 0, "rt/odometer_state received at least once");
#ifdef K1SIM_CHECK_BATTERY
    std::printf("battery_state: %llu samples in window, last soc=%.1f\n",
                static_cast<unsigned long long>(g_battery_count.load()),
                static_cast<double>(g_battery_soc.load()));
    check(g_battery_count.load() > 0, "rt/battery_state received at least once (1 Hz publisher)");
    check(g_battery_soc.load() > 0.0f && g_battery_soc.load() <= 100.0f,
          "rt/battery_state soc in (0, 100]");
#else
    std::printf("[SKIP] rt/battery_state checks (SDK build lacks booster/idl/b1/BatteryState.h — "
                "set BOOSTER_SDK_ROOT to a pinned-SDK (324946e7) extract to enable)\n");
#endif

    booster::robot::b1::B1LocoClient client;
    client.Init();

    std::printf("RPC round trip (each must return 0 within 1000 ms):\n");
    timed_rpc("ChangeMode(kPrepare)",
              [&] { return client.ChangeMode(booster::robot::RobotMode::kPrepare); });

    // Give the mode change a couple of SimStateUpdate ticks (50 Hz) to propagate
    // into SdkBridge's cached mode before asking for it back.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    {
        booster::robot::b1::GetModeResponse mode_resp;
        const auto start = std::chrono::steady_clock::now();
        const int32_t ret = client.GetMode(mode_resp);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        std::printf("  GetMode() -> ret=%d, mode=%d, latency=%.2f ms\n",
                    ret,
                    static_cast<int>(mode_resp.mode_),
                    ms);
        check(ret == 0, "GetMode() returned 0");
        check(ms < 1000.0, "GetMode() completed within 1000 ms");
        check(mode_resp.mode_ == booster::robot::RobotMode::kPrepare,
              "GetMode() reports kPrepare after ChangeMode(kPrepare)");
    }

    timed_rpc("Move(0.1, 0, 0)", [&] { return client.Move(0.1f, 0.0f, 0.0f); });
    timed_rpc("RotateHead(0.2, 0.3)", [&] { return client.RotateHead(0.2f, 0.3f); });
    timed_rpc("GetUp()", [&] { return client.GetUp(); });

    std::printf("\n=== %s (%d check%s failed) ===\n",
                g_failures == 0 ? "PASS" : "FAIL",
                g_failures,
                g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
