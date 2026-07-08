// Standalone binary for the M4/M5 contract test: ConsoleLog + the real
// module::SdkBridge + a synthetic SimStateUpdate source (SyntheticState), with no
// dependency on module::Simulation/Locomotion/Viewer. See SyntheticState.hpp for
// rationale. Not part of k1_mujoco_sim.
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

    // NUClear's PowerPlant does not auto-install its built-in extensions — ChronoController
    // is what actually drives on<Every<...>> (SyntheticState's 50 Hz tick, SdkBridge's 1 Hz
    // battery publish); without it those reactions are bound but never scheduled (empirically
    // verified: silently inert, no error/log). k1_mujoco_sim's own src/main.cpp (frozen, not
    // owned by this workstream) needs the same install — flagged to the lead, see workstream
    // C's final report.
    plant.install<NUClear::extension::ChronoController>();

    plant.install<k1sim::module::ConsoleLog>();
    plant.install<k1sim::module::SdkBridge>();
    plant.install<k1sim::module::sdkbridge::test_support::SyntheticState>();

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    plant.start();
    return 0;
}
