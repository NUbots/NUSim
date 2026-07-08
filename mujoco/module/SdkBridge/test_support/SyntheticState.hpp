#ifndef K1SIM_MODULE_SDKBRIDGE_TEST_SUPPORT_SYNTHETICSTATE_HPP
#define K1SIM_MODULE_SDKBRIDGE_TEST_SUPPORT_SYNTHETICSTATE_HPP

#include <mutex>
#include <nuclear>

// Test-only reactor that stands in for module::Simulation + module::Locomotion so
// the M4/M5 contract test (test/contract/test_sdk_roundtrip.py) can exercise
// module::SdkBridge's full DDS surface without depending on those workstreams'
// completion timing. Emits a plausible k1sim::message::SimStateUpdate at 50 Hz and
// reacts to the NUClear messages module::SdkBridge's RpcServer emits (mode changes,
// walk/head commands, get up/lie down/kick requests) closely enough to make the
// contract test meaningful: mode tracks ChangeMode, odometer integrates Move
// velocities, head joints track RotateHead. Not a physics simulator — no dynamics,
// no PD control, no gait. Only linked into the `sdkbridge_synthetic_sim` test
// binary (module/SdkBridge/test_support/CMakeLists.txt); never linked into
// k1_mujoco_sim.

namespace k1sim::module::sdkbridge::test_support {

class SyntheticState : public NUClear::Reactor {
public:
    explicit SyntheticState(std::unique_ptr<NUClear::Environment> environment);

private:
    std::mutex mutex_;
    double sim_time_    = 0.0;
    uint64_t step_count_ = 0;

    int mode_       = 0;  // booster::RobotMode::DAMPING
    int fall_state_ = 0;  // booster::FallState::IS_READY

    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;  // planar odometry, integrated from WalkCommand
    double vx_ = 0.0, vy_ = 0.0, vyaw_ = 0.0;

    double head_pitch_ = 0.0, head_yaw_ = 0.0;
};

}  // namespace k1sim::module::sdkbridge::test_support

#endif  // K1SIM_MODULE_SDKBRIDGE_TEST_SUPPORT_SYNTHETICSTATE_HPP
