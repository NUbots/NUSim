#ifndef K1SIM_MODULE_SIMULATION_HPP
#define K1SIM_MODULE_SIMULATION_HPP

#include <memory>
#include <nuclear>

#include "module/Simulation/src/SimCore.hpp"

namespace k1sim::module {

// Thin NUClear wrapper around SimCore: loads config, loads the model, emits SimHandles once
// (Startup) and SimStateUpdate at 50 Hz (from the physics thread), starts the physics thread
// immediately with the PD-to-ready fallback engaged, and atomically swaps in the
// Locomotion-provided StepController whenever a ControllerHandle arrives.
class Simulation : public NUClear::Reactor {
public:
    explicit Simulation(std::unique_ptr<NUClear::Environment> environment);

private:
    // Constructed in the Reactor constructor (not inside on<Startup>) so that its atomic
    // controller_ member always exists before any reaction can run — on<Startup> handlers
    // across reactors are not guaranteed to be ordered/serialized by NUClear, so
    // on<Trigger<ControllerHandle>> could otherwise race the construction of sim_.
    std::unique_ptr<SimCore> sim_;
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_SIMULATION_HPP
