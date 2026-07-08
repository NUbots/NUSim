#include "module/Locomotion/src/Locomotion.hpp"

#include "shared/k1/BoosterApi.hpp"
#include "shared/message/Commands.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

using message::ControllerHandle;
using message::GetUpRequest;
using message::HeadCommand;
using message::LieDownRequest;
using message::LowCmdMessage;
using message::ModeChangeRequest;
using message::VisualKickRequest;
using message::WalkCommand;

Locomotion::Locomotion(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Startup>().then([this] {
        auto locomotion_cfg = config::load("locomotion.yaml");
        auto gains_cfg       = config::load("gains.yaml");

        const auto backend = locomotion_cfg["backend"].as<std::string>("kinematic");
        log<NUClear::LogLevel::INFO>("Locomotion starting (backend:", backend, ")");

        try {
            controller_ = std::make_unique<LocomotionController>(locomotion_cfg, gains_cfg);
        }
        catch (const std::exception& e) {
            log<NUClear::LogLevel::FATAL>("Locomotion: failed to build the mode controller/backend:", e.what());
            throw;
        }

        emit(std::make_unique<ControllerHandle>(ControllerHandle{controller_.get()}));
        log<NUClear::LogLevel::INFO>("Locomotion ready (backend:", backend, ")");
    });

    on<Trigger<WalkCommand>>().then([this](const WalkCommand& cmd) {
        controller_->set_walk_command(cmd.vx, cmd.vy, cmd.vyaw);
    });

    on<Trigger<HeadCommand>>().then([this](const HeadCommand& cmd) {
        controller_->set_head_command(cmd.pitch, cmd.yaw);
    });

    on<Trigger<ModeChangeRequest>>().then([this](const ModeChangeRequest& req) {
        log<NUClear::LogLevel::INFO>("ChangeMode requested: mode", req.mode);
        controller_->request_mode_change(req.mode);
    });

    on<Trigger<GetUpRequest>>().then([this](const GetUpRequest& req) {
        log<NUClear::LogLevel::INFO>("GetUp requested (target mode", req.target_mode, ")");
        controller_->request_get_up(req.target_mode);
    });

    on<Trigger<LieDownRequest>>().then([this](const LieDownRequest&) {
        log<NUClear::LogLevel::INFO>("LieDown requested");
        controller_->request_lie_down();
    });

    on<Trigger<VisualKickRequest>>().then([this](const VisualKickRequest& req) {
        if (req.start) {
            log<NUClear::LogLevel::INFO>("VisualKick requested (version", req.version, ")");
            controller_->request_kick(req.version);
        }
    });

    on<Trigger<LowCmdMessage>>().then([this](const LowCmdMessage& cmd) {
        controller_->set_low_cmd(cmd.cmd_type, cmd.motors);
    });

    on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Locomotion shutting down"); });
}

}  // namespace k1sim::module
