#include "module/SdkBridge/src/SdkBridge.hpp"

#include <cstdlib>
#include <string>

#include "shared/message/Commands.hpp"
#include "shared/message/SimMessages.hpp"
#include "shared/util/Config.hpp"

namespace k1sim::module {

namespace {
bool env_flag_set(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && std::string(value) != "0" && std::string(value) != "";
}
}  // namespace

SdkBridge::SdkBridge(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Startup>().then([this] {
        auto cfg = config::load("dds.yaml");

        const int domain                 = cfg["domain"].as<int>(0);
        const bool udp_only               = cfg["udp_only"].as<bool>(false) || env_flag_set("K1_DDS_UDP_ONLY");
        const double battery_soc          = cfg["battery_soc"].as<double>(100.0);
        const int64_t unknown_api_status = cfg["unknown_api_status"].as<int64_t>(0);

        dds_             = std::make_unique<sdkbridge::DdsParticipant>(domain, udp_only);
        state_publisher_ = std::make_unique<sdkbridge::StatePublisher>(*dds_, battery_soc);
        rpc_server_      = std::make_unique<sdkbridge::RpcServer>(*dds_, *this, unknown_api_status);

        log<NUClear::LogLevel::INFO>("SdkBridge ready (DDS domain",
                                      domain,
                                      udp_only ? "UDP-only" : "UDP+SHM",
                                      ")");
    });

    // 50 Hz (matches SimCore's state_publish_divisor) — write low_state/odometer_state
    // every tick, fall_down on change/keepalive, and cache the mode for GET_MODE.
    on<Trigger<message::SimStateUpdate>>().then([this](const message::SimStateUpdate& update) {
        if (!rpc_server_ || !state_publisher_) {
            return;  // race with on<Startup> — harmless, next tick will publish
        }
        rpc_server_->set_current_mode(update.mode);
        state_publisher_->publish(update);
    });

    on<Every<1, std::chrono::seconds>>().then([this] {
        if (state_publisher_) {
            state_publisher_->publish_battery();
        }
    });

    on<Shutdown>().then([this] {
        log<NUClear::LogLevel::INFO>("SdkBridge shutting down");
        // Destroy in reverse-dependency order: readers/writers before the participant.
        rpc_server_.reset();
        state_publisher_.reset();
        dds_.reset();
    });
}

}  // namespace k1sim::module
