// Contract-test binary: ConsoleLog + the real module::SdkBridge + a synthetic SimStateUpdate source
// (SyntheticState), without Simulation/Locomotion/Viewer. Not part of k1_mujoco_sim.
#include <csignal>
#include <nuclear>

#include "module/ConsoleLog/src/ConsoleLog.hpp"
#include "module/SdkBridge/src/SdkBridge.hpp"
#include "module/SdkBridge/test_support/SyntheticState.hpp"

namespace {
    void handle_signal(int /*signum*/) {
        if (NUClear::PowerPlant::powerplant != nullptr) {
            NUClear::PowerPlant::powerplant->shutdown();
        }
    }
}  // namespace

int main() {
    NUClear::Configuration config;
    config.default_pool_concurrency = 2;
    NUClear::PowerPlant plant(config);

    // PowerPlant doesn't install ChronoController by default, and without it on<Every<...>>
    // reactions (SyntheticState's 50 Hz tick, SdkBridge's 1 Hz battery) silently never run.
    plant.install<NUClear::extension::ChronoController>();

    plant.install<k1sim::module::ConsoleLog>();
    plant.install<k1sim::module::SdkBridge>();
    plant.install<k1sim::module::sdkbridge::test_support::SyntheticState>();

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    plant.start();
    return 0;
}
