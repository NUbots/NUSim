#ifndef K1SIM_MODULE_LOCOMOTION_HPP
#define K1SIM_MODULE_LOCOMOTION_HPP

#include <memory>
#include <nuclear>

#include "module/Locomotion/src/LocomotionController.hpp"

namespace k1sim::module {

// Reduced mode state machine (Damping/Prepare/Custom). Emits ControllerHandle;
// consumes HeadCommand/ModeChangeRequest/LowCmdMessage and forwards each into
// LocomotionController's thread-safe setters. Locomotion *policies* live in
// NUbots_K1 and reach the sim as LowCmd servo targets; the old high-level RPCs
// (Move/GetUp/LieDown/VisualKick) are accepted on the wire but ignored with a
// warning. The FSM logic itself lives in LocomotionController (deliberately
// NUClear-free, see that header); this reactor only wires it to NUClear and
// logs incoming requests.
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
