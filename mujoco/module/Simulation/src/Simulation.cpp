#include "module/Simulation/src/Simulation.hpp"

#include <cstddef>
#include <mujoco/mujoco.h>
#include <string>
#include <utility>

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
            cfg.model_path = !cli().model.empty() ? cli().model : config::field_scene(sim_cfg, cli().field);
            cfg.initial_keyframe =
                !cli().keyframe.empty() ? cli().keyframe : sim_cfg["initial_keyframe"].as<std::string>("ready");
            // CliOptions.rtf < 0 means "use config"; the config's real_time_factor may itself be 0
            // (free-run) — SimCore treats rtf <= 0 as free-run.
            cfg.rtf                   = cli().rtf >= 0.0 ? cli().rtf : sim_cfg["real_time_factor"].as<double>(1.0);
            cfg.robots                = cli().robots;
            cfg.state_publish_divisor = sim_cfg["state_publish_divisor"].as<int>(20);
            cfg.resync_threshold      = sim_cfg["resync_threshold"].as<double>(0.05);

            const auto surface           = sim_cfg["surface"];
            cfg.surface.enabled          = surface["enabled"].as<bool>(false);
            cfg.surface.friction         = surface["friction"].as<double>(0.8);
            cfg.surface.solref_timeconst = surface["solref_timeconst"].as<double>(0.02);
            cfg.surface.solref_dampratio = surface["solref_dampratio"].as<double>(1.0);

            cfg.foot_log_path = sim_cfg["foot_log"].as<std::string>("");

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
        SimCore::Config sim_config = build_sim_config();
        const std::string scene    = sim_config.model_path;
        sim_                       = std::make_unique<SimCore>(std::move(sim_config),
                                         [this](std::unique_ptr<message::SimStateUpdate> state) { emit(state); });

        on<Startup>().then([this, scene] {
            sim_->load_model();

            auto handles          = std::make_unique<message::SimHandles>();
            handles->model        = sim_->model();
            handles->data         = sim_->data();
            handles->mutex        = &sim_->mutex();
            handles->measured_rtf = &sim_->measured_rtf();
            emit(handles);

            log<NUClear::LogLevel::INFO>("Simulation ready (scene",
                                         scene,
                                         "— MuJoCo",
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

        // Viewer Backspace (or any other emitter): snap mjData back to the startup keyframe.
        // Physics state only — the attached controller keeps its mode/commands (SimCore::reset).
        on<Trigger<message::SimResetRequest>>().then([this] {
            sim_->reset();
            log<NUClear::LogLevel::INFO>("Simulation: state reset to startup keyframe");
        });

        // Base-pose heartbeat: one INFO line every ~5 s of sim time (updates arrive at
        // 50 Hz), so headless runs show whether the robot is actually moving.
        on<Trigger<message::SimStateUpdate>>().then([this](const message::SimStateUpdate& s) {
            if (s.sim_time >= next_pose_log_) {
                next_pose_log_ = s.sim_time + 5.0;
                log<NUClear::LogLevel::INFO>("Simulation: t =",
                                             s.sim_time,
                                             "s, base x =",
                                             s.base.x,
                                             "y =",
                                             s.base.y,
                                             "z =",
                                             s.base.z,
                                             ", mode =",
                                             s.mode);
            }
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
