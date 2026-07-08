#ifndef K1SIM_MODULE_CONSOLELOG_HPP
#define K1SIM_MODULE_CONSOLELOG_HPP

#include <nuclear>

namespace k1sim::module {

// Minimal console sink for NUClear log() messages (NUClear emits LogMessage;
// without a handler reactor, logs go nowhere).
class ConsoleLog : public NUClear::Reactor {
public:
    explicit ConsoleLog(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_CONSOLELOG_HPP
