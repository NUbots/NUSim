#include "module/Simulation/src/Simulation.hpp"

#include <cstddef>
#include <mujoco/mujoco.h>

#include "shared/CliOptions.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

namespace {

SimCore::Config build_sim_config() {
    auto sim_cfg   = config::load("simulation.yaml");
    auto gains_cfg = config::load("gains.yaml");

    SimCore::Config cfg;
    cfg.model_path = !cli().model.empty() ? cli().model : sim_cfg["model"].as<std::string>();
    // CliOptions.rtf < 0 means "use config"; the config's real_time_factor may itself be 0
    // (free-run) — SimCore treats rtf <= 0 as free-run.
    cfg.rtf                   = cli().rtf >= 0.0 ? cli().rtf : sim_cfg["real_time_factor"].as<double>(1.0);
    cfg.state_publish_divisor = sim_cfg["state_publish_divisor"].as<int>(20);
    cfg.resync_threshold      = sim_cfg["resync_threshold"].as<double>(0.05);

    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        cfg.kp[i]                  = gains_cfg["kp"][i].as<double>();
        cfg.kd[i]                  = gains_cfg["kd"][i].as<double>();
        cfg.ready_pose_fallback[i] = gains_cfg["ready_pose"][i].as<double>();
    }
    return cfg;
}

}  // namespace

Simulation::Simulation(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    // Constructed here (not inside on<Startup>) — see the header comment on sim_.
    sim_ = std::make_unique<SimCore>(build_sim_config(),
                                     [this](std::unique_ptr<message::SimStateUpdate> state) { emit(state); });

    on<Startup>().then([this] {
        sim_->load_model();

        auto handles          = std::make_unique<message::SimHandles>();
        handles->model        = sim_->model();
        handles->data         = sim_->data();
        handles->mutex        = &sim_->mutex();
        handles->measured_rtf = &sim_->measured_rtf();
        emit(handles);

        log<NUClear::LogLevel::INFO>("Simulation ready (MuJoCo",
                                      mj_versionString(),
                                      "— nq:",
                                      sim_->model()->nq,
                                      "nu:",
                                      sim_->model()->nu,
                                      ") — starting physics thread (PD-to-ready fallback until a "
                                      "controller attaches)");

        // Start immediately with the PD fallback engaged; if Locomotion's ControllerHandle
        // arrives it is swapped in atomically by the Trigger reaction below — race-free, no
        // need to wait for it (Locomotion may not even be installed, e.g. in unit tests).
        sim_->start();
    });

    on<Trigger<message::ControllerHandle>>().then([this](const message::ControllerHandle& handle) {
        sim_->set_controller(handle.controller);
        log<NUClear::LogLevel::INFO>("Simulation: controller attached");
    });

    on<Shutdown>().then([this] {
        log<NUClear::LogLevel::INFO>("Simulation shutting down (measured RTF",
                                      sim_->measured_rtf().load(),
                                      ", dropped deadlines:",
                                      sim_->dropped_deadlines(),
                                      ")");
        sim_->stop();
        sim_->unload();
    });
}

}  // namespace k1sim::module
