#ifndef K1SIM_MODULE_VIEWER_HPP
#define K1SIM_MODULE_VIEWER_HPP

#include <nuclear>

namespace k1sim::module {

// GLFW window + MuJoCo mjv/mjr GPU rendering, on NUClear's MainThread pool
// (GLFW requires the true main thread; PowerPlant::start() runs MainThread
// reactions there). Skipped entirely under --headless.
// Implementation lands with workstream E (M3); this is the M0 stub.
class Viewer : public NUClear::Reactor {
public:
    explicit Viewer(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_VIEWER_HPP
