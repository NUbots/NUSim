#ifndef K1SIM_MODULE_LOCOMOTION_HPP
#define K1SIM_MODULE_LOCOMOTION_HPP

#include <memory>
#include <nuclear>

#include "module/Locomotion/src/LocomotionController.hpp"

namespace k1sim::module {

// Mode state machine (Damping/Prepare/Walking=Soccer/Custom + GetUp/LieDown/
// Kick transients) and the pluggable locomotion backends. Emits
// ControllerHandle; consumes WalkCommand/HeadCommand/ModeChangeRequest/
// GetUpRequest/LieDownRequest/VisualKickRequest/LowCmdMessage and forwards
// each into LocomotionController's thread-safe setters. The FSM logic itself
// lives in LocomotionController (deliberately NUClear-free, see that
// header); this reactor only wires it to NUClear and logs incoming requests.
class Locomotion : public NUClear::Reactor {
public:
    explicit Locomotion(std::unique_ptr<NUClear::Environment> environment);

private:
    std::unique_ptr<LocomotionController> controller_;
};

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_HPP
