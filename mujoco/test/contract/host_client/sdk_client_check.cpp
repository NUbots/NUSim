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
//   - rt/head_pose received at least once with a finite, standing-height pose (the
//     topic NUbots' K1Sensors places the camera and torso from). Only when the SDK
//     ships booster/idl/geometry_msgs/Pose.h; SKIP printed otherwise.
//   - rt/odom (nav_msgs Odometry, the source of NUbots' Sensors.vTw) received at
//     least once with finite pose and twist, reporting its frames. Only when the SDK
//     ships booster/idl/nav_msgs/Odometry.h (1.7.0 firmware); SKIP printed otherwise.
//   - rt/battery_state received at least once during the 5s window (published at
//     1 Hz via NUClear on<Every<1, std::chrono::seconds>>) with soc in (0, 100].
//     Only when the SDK being built against ships booster/idl/b1/BatteryState.h —
//     the pinned 324946e7 does, the older local checkout (7fb7287) doesn't even
//     compile the type into its .a; guarded with __has_include, SKIP printed
//     otherwise. Point BOOSTER_SDK_ROOT at a pinned-SDK extract to enable it.
//   - rt/boostercamera/head/rgb (sensor_msgs Image) received at >= 20 frames/s as whole
//     rgb8 frames stamped within 1 s of our clock (K1Camera's MAX_CLOCK_SKEW), and its
//     /camera_info with a usable intrinsic matrix -- the pair NUbots' K1Camera subscribes
//     to. Only when the SDK ships booster/idl/sensor_msgs/Image.h; SKIP printed otherwise,
//     or with --no-camera (the synthetic sim renders no camera).
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
#include <cstring>
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
    #define K1SIM_SDK_PINNED    1
#endif
#if __has_include(<booster/idl/geometry_msgs/Pose.h>)
    #include <booster/idl/geometry_msgs/Pose.h>
    #define K1SIM_CHECK_HEAD_POSE 1
#endif
#if __has_include(<booster/idl/nav_msgs/Odometry.h>)
    #include <booster/idl/nav_msgs/Odometry.h>
    #define K1SIM_CHECK_ROS_ODOMETRY 1
#endif
#if __has_include(<booster/idl/sensor_msgs/Image.h>)
    #include <booster/idl/sensor_msgs/CameraInfo.h>
    #include <booster/idl/sensor_msgs/Image.h>
    #define K1SIM_CHECK_CAMERA 1
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

#ifdef K1SIM_CHECK_HEAD_POSE
    std::atomic<uint64_t> g_head_pose_count{0};
    std::atomic<double> g_head_pose_z{-1.0};
    std::atomic<bool> g_head_pose_finite{true};
    void HeadPoseHandler(const void* msg) {
        const auto* pose = static_cast<const geometry_msgs::msg::Pose*>(msg);
        const auto& p    = pose->position();
        const auto& q    = pose->orientation();
        g_head_pose_count.fetch_add(1, std::memory_order_relaxed);
        g_head_pose_z.store(p.z(), std::memory_order_relaxed);
        for (double v : {p.x(), p.y(), p.z(), q.x(), q.y(), q.z(), q.w()}) {
            if (!std::isfinite(v)) {
                g_head_pose_finite.store(false, std::memory_order_relaxed);
            }
        }
    }
#endif

#ifdef K1SIM_CHECK_ROS_ODOMETRY
    std::atomic<uint64_t> g_ros_odom_count{0};
    std::atomic<bool> g_ros_odom_finite{true};
    std::string g_ros_odom_frames;  // written once, before the count is first incremented
    void RosOdometryHandler(const void* msg) {
        const auto* odom  = static_cast<const nav_msgs::msg::Odometry*>(msg);
        const auto& p     = odom->pose().pose().position();
        const auto& twist = odom->twist().twist();
        if (g_ros_odom_count.load() == 0) {
            g_ros_odom_frames = "'" + odom->header().frame_id() + "' -> '" + odom->child_frame_id() + "'";
        }
        for (double v : {p.x(),
                         p.y(),
                         p.z(),
                         twist.linear().x(),
                         twist.linear().y(),
                         twist.linear().z(),
                         twist.angular().x(),
                         twist.angular().y(),
                         twist.angular().z()}) {
            if (!std::isfinite(v)) {
                g_ros_odom_finite.store(false, std::memory_order_relaxed);
            }
        }
        g_ros_odom_count.fetch_add(1, std::memory_order_relaxed);
    }
#endif

#ifdef K1SIM_CHECK_CAMERA
    std::atomic<uint64_t> g_image_count{0};
    std::atomic<bool> g_image_whole{true};
    std::atomic<double> g_image_max_skew_s{0.0};
    std::atomic<bool> g_image_described{false};
    std::string g_image_desc;  // written once, by the first frame
    void ImageHandler(const void* msg) {
        const auto* image = static_cast<const sensor_msgs::msg::Image*>(msg);
        if (image->encoding() != "rgb8" || image->step() != image->width() * 3
            || image->data().size() != std::size_t(image->step()) * image->height() || image->width() == 0) {
            g_image_whole.store(false, std::memory_order_relaxed);
        }
        const auto now     = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        const double stamp = image->header().stamp().sec() + image->header().stamp().nanosec() * 1e-9;
        if (std::fabs(now - stamp) > g_image_max_skew_s.load()) {
            g_image_max_skew_s.store(std::fabs(now - stamp), std::memory_order_relaxed);
        }
        if (!g_image_described.exchange(true)) {
            g_image_desc = std::to_string(image->width()) + "x" + std::to_string(image->height()) + " "
                           + image->encoding() + ", frame '" + image->header().frame_id() + "'";
        }
        g_image_count.fetch_add(1, std::memory_order_relaxed);
    }

    std::atomic<uint64_t> g_camera_info_count{0};
    std::atomic<double> g_camera_info_fx{0.0};
    std::atomic<bool> g_camera_info_matches_image{true};
    void CameraInfoHandler(const void* msg) {
        const auto* info = static_cast<const sensor_msgs::msg::CameraInfo*>(msg);
        g_camera_info_fx.store(info->k()[0], std::memory_order_relaxed);
        if (info->width() == 0 || info->height() == 0 || info->k()[8] != 1.0) {
            g_camera_info_matches_image.store(false, std::memory_order_relaxed);
        }
        g_camera_info_count.fetch_add(1, std::memory_order_relaxed);
    }
#endif

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
        const auto start  = std::chrono::steady_clock::now();
        const int32_t ret = fn();
        const auto end    = std::chrono::steady_clock::now();
        const double ms   = std::chrono::duration<double, std::milli>(end - start).count();
        std::printf("  %s -> ret=%d, latency=%.2f ms\n", name, ret, ms);
        check(ret == 0, std::string(name) + " returned 0");
        check(ms < 1000.0, std::string(name) + " completed within 1000 ms");
    }

}  // namespace

int main(int argc, char** argv) {
    std::printf("=== k1sim SdkBridge contract test (host SDK client) ===\n");
    const bool check_camera = !(argc > 1 && std::strcmp(argv[1], "--no-camera") == 0);

#ifdef K1SIM_SDK_PINNED
    ChannelFactory::Instance()->InitDefault(0);
#else
    ChannelFactory::Instance()->Init(0);
#endif

    ChannelSubscriber<LowState> low_state_sub(booster::robot::b1::kTopicLowState, LowStateHandler);
    low_state_sub.InitChannel();
    ChannelSubscriber<Odometer> odom_sub(booster::robot::b1::kTopicOdometerState, OdometerHandler);
    odom_sub.InitChannel();
#ifdef K1SIM_CHECK_HEAD_POSE
    ChannelSubscriber<geometry_msgs::msg::Pose> head_pose_sub("rt/head_pose", HeadPoseHandler);
    head_pose_sub.InitChannel();
#endif
#ifdef K1SIM_CHECK_ROS_ODOMETRY
    ChannelSubscriber<nav_msgs::msg::Odometry> ros_odom_sub(booster::robot::b1::kTopicRosOdometer, RosOdometryHandler);
    ros_odom_sub.InitChannel();
#endif
#ifdef K1SIM_CHECK_BATTERY
    ChannelSubscriber<BatteryState> battery_sub("rt/battery_state", BatteryHandler);
    battery_sub.InitChannel();
#endif
#ifdef K1SIM_CHECK_CAMERA
    ChannelSubscriber<sensor_msgs::msg::Image> image_sub("rt/boostercamera/head/rgb", ImageHandler);
    ChannelSubscriber<sensor_msgs::msg::CameraInfo> camera_info_sub("rt/boostercamera/head/rgb/camera_info",
                                                                    CameraInfoHandler);
    if (check_camera) {
        image_sub.InitChannel();
        camera_info_sub.InitChannel();
    }
#endif

    // Let DDS discovery settle before measuring rate.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    g_low_state_count.store(0);
    g_odom_count.store(0);
#ifdef K1SIM_CHECK_BATTERY
    g_battery_count.store(0);
#endif
#ifdef K1SIM_CHECK_CAMERA
    g_image_count.store(0);
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
#ifdef K1SIM_CHECK_HEAD_POSE
    std::printf("head_pose: %llu samples in window, last z=%.3f\n",
                static_cast<unsigned long long>(g_head_pose_count.load()),
                g_head_pose_z.load());
    check(g_head_pose_count.load() > 0, "rt/head_pose received at least once");
    check(g_head_pose_finite.load(), "rt/head_pose values are finite");
    check(g_head_pose_z.load() > 0.5 && g_head_pose_z.load() < 1.2,
          "rt/head_pose z is a standing head height above the footprint (0.5, 1.2) m");
#else
    std::printf("[SKIP] rt/head_pose checks (SDK build lacks booster/idl/geometry_msgs/Pose.h)\n");
#endif
#ifdef K1SIM_CHECK_ROS_ODOMETRY
    std::printf("odom: %llu samples in window, frames %s\n",
                static_cast<unsigned long long>(g_ros_odom_count.load()),
                g_ros_odom_frames.c_str());
    check(g_ros_odom_count.load() > 0, "rt/odom received at least once");
    check(g_ros_odom_finite.load(), "rt/odom pose and twist are finite");
#else
    std::printf("[SKIP] rt/odom checks (SDK build lacks booster/idl/nav_msgs/Odometry.h)\n");
#endif
#ifdef K1SIM_CHECK_BATTERY
    std::printf("battery_state: %llu samples in window, last soc=%.1f\n",
                static_cast<unsigned long long>(g_battery_count.load()),
                static_cast<double>(g_battery_soc.load()));
    check(g_battery_count.load() > 0, "rt/battery_state received at least once (1 Hz publisher)");
    check(g_battery_soc.load() > 0.0f && g_battery_soc.load() <= 100.0f, "rt/battery_state soc in (0, 100]");
#else
    std::printf(
        "[SKIP] rt/battery_state checks (SDK build lacks booster/idl/b1/BatteryState.h — "
        "set BOOSTER_SDK_ROOT to a pinned-SDK (324946e7) extract to enable)\n");
#endif

#ifdef K1SIM_CHECK_CAMERA
    if (check_camera) {
        const double fps = static_cast<double>(g_image_count.load()) / elapsed_s;
        std::printf("camera: %.2f frames/s (%s), max stamp skew %.3f s; camera_info: %llu samples, fx=%.1f\n",
                    fps,
                    g_image_desc.c_str(),
                    g_image_max_skew_s.load(),
                    static_cast<unsigned long long>(g_camera_info_count.load()),
                    g_camera_info_fx.load());
        check(fps >= 20.0, "rt/boostercamera/head/rgb received at >= 20 frames/s");
        check(g_image_whole.load(), "rt/boostercamera/head/rgb frames are whole rgb8 images");
        check(g_image_max_skew_s.load() < 1.0, "rt/boostercamera/head/rgb stamps within 1 s of our clock");
        check(g_camera_info_count.load() > 0, "rt/boostercamera/head/rgb/camera_info received at least once");
        check(g_camera_info_fx.load() > 0.0 && g_camera_info_matches_image.load(),
              "rt/boostercamera/head/rgb/camera_info has a usable intrinsic matrix");
    }
    else {
        std::printf("[SKIP] camera checks (--no-camera)\n");
    }
#else
    std::printf("[SKIP] camera checks (SDK build lacks booster/idl/sensor_msgs/Image.h)\n");
#endif

    booster::robot::b1::B1LocoClient client;
    client.Init();

    std::printf("RPC round trip (each must return 0 within 1000 ms):\n");
    timed_rpc("ChangeMode(kPrepare)", [&] { return client.ChangeMode(booster::robot::RobotMode::kPrepare); });

    // Give the mode change a couple of SimStateUpdate ticks (50 Hz) to propagate
    // into SdkBridge's cached mode before asking for it back.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    {
        booster::robot::b1::GetModeResponse mode_resp;
        const auto start  = std::chrono::steady_clock::now();
        const int32_t ret = client.GetMode(mode_resp);
        const double ms   = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        std::printf("  GetMode() -> ret=%d, mode=%d, latency=%.2f ms\n", ret, static_cast<int>(mode_resp.mode_), ms);
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
