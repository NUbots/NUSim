#include "module/ConsoleLog/src/ConsoleLog.hpp"

#include <cstdio>
#include <string>

namespace k1sim::module {

ConsoleLog::ConsoleLog(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

    on<Trigger<NUClear::message::LogMessage>>().then([](const NUClear::message::LogMessage& msg) {
        if (msg.level < msg.display_level) {
            return;
        }
        const std::string level = msg.level;  // LogLevel has a string conversion operator
        std::printf("[%s] %s: %s\n", level.c_str(), msg.reactor_name.c_str(), msg.message.c_str());
        std::fflush(stdout);
    });
}

}  // namespace k1sim::module
