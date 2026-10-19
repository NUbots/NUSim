#ifndef K1SIM_MODULE_LOCOMOTION_HPP
#define K1SIM_MODULE_LOCOMOTION_HPP

#include <memory>
#include <nuclear>

#include "module/Locomotion/src/LocomotionController.hpp"

namespace k1sim::module {

    // Wires LocomotionController (the Damping/Prepare/Custom FSM) to NUClear: emits
    // ControllerHandle, forwards HeadCommand/ModeChangeRequest/LowCmdMessage to its setters, and logs
    // requests. Move/GetUp/LieDown/VisualKick are accepted on the wire but ignored with a warning.
    class Locomotion : public NUClear::Reactor {
    public:
        explicit Locomotion(std::unique_ptr<NUClear::Environment> environment);

    private:
        void warn_once(bool& flag, const char* rpc);

        std::unique_ptr<LocomotionController> controller_;
        bool walk_warned_    = false;
        bool getup_warned_   = false;
        bool liedown_warned_ = false;
        bool kick_warned_    = false;
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_HPP
