#include "module/Supervisor/src/Supervisor.hpp"

#include <vector>

#include "module/Supervisor/src/GameControllerPacket.hpp"
#include "module/Supervisor/src/SupervisorConfig.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

    namespace gc = k1sim::module::supervisor::gc;
    using k1sim::module::supervisor::SupervisorConfig;
    using k1sim::module::supervisor::SupervisorLogic;

    namespace {

        SupervisorConfig build_config() {
            return k1sim::module::supervisor::load_config(config::load("supervisor.yaml"));
        }

    }  // namespace

    Supervisor::Supervisor(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        // Loaded in the constructor, not on<Startup>, so the port is known before on<UDP> binds.
        SupervisorConfig cfg = build_config();
        const bool enabled   = cfg.enabled;
        const int gc_port    = cfg.gc_port;
        logic_               = std::make_unique<SupervisorLogic>(std::move(cfg));

        on<Trigger<message::SimHandles>>().then([this](const message::SimHandles& handles) {
            model_.store(handles.model, std::memory_order_release);
            data_.store(handles.data, std::memory_order_release);
            sim_mutex_.store(handles.mutex, std::memory_order_release);
        });

        if (!enabled) {
            on<Startup>().then(
                [this] { log<NUClear::LogLevel::INFO>("Supervisor disabled (supervisor.yaml: enabled: false)"); });
            on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Supervisor shutting down"); });
            return;
        }

        // Plain on<UDP> bound to any address, not on<UDP::Broadcast>: that only accepts datagrams
        // sent to a broadcast address, so a test packet to 127.0.0.1 would never arrive. INADDR_ANY
        // still receives real LAN broadcasts.
        // Binding throws if the port is taken (another sim, or a GameController listener on this
        // host); log and run without a supervisor rather than crash.
        try {
            on<UDP, Single>(gc_port).then([this](const UDP::Packet& packet) {
                gc::GameControllerPacket parsed{};
                if (!gc::try_parse(packet.payload.data(), packet.payload.size(), parsed)) {
                    // Not a recognised GameController packet; unrelated traffic is normal.
                    return;
                }

                const mjModel* m      = model_.load(std::memory_order_acquire);
                mjData* d             = data_.load(std::memory_order_acquire);
                std::mutex* sim_mutex = sim_mutex_.load(std::memory_order_acquire);
                if (m == nullptr || d == nullptr || sim_mutex == nullptr) {
                    log<NUClear::LogLevel::WARN>(
                        "GameController packet received before the sim model was ready — ignoring");
                    return;
                }

                std::vector<SupervisorLogic::Action> actions;
                {
                    std::lock_guard<std::mutex> lock(*sim_mutex);
                    actions = logic_->process(m, d, parsed);
                }
                for (const auto& action : actions) {
                    if (action.level == SupervisorLogic::Action::Level::WARN) {
                        log<NUClear::LogLevel::WARN>(action.message.c_str());
                    }
                    else {
                        log<NUClear::LogLevel::INFO>(action.message.c_str());
                    }
                }
            });
        }
        catch (const std::exception& e) {
            log<NUClear::LogLevel::WARN>("Supervisor: could not bind UDP port",
                                         gc_port,
                                         "(",
                                         e.what(),
                                         ") — running without GameController placement");
        }

        on<Startup>().then([this, gc_port] {
            log<NUClear::LogLevel::INFO>("Supervisor ready — waiting for GameController broadcasts on port", gc_port);
        });

        on<Shutdown>().then([this] { log<NUClear::LogLevel::INFO>("Supervisor shutting down"); });
    }

}  // namespace k1sim::module
