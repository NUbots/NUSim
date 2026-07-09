#ifndef K1SIM_MODULE_LOCOMOTION_BACKENDS_POLICYBACKEND_HPP
#define K1SIM_MODULE_LOCOMOTION_BACKENDS_POLICYBACKEND_HPP

// Only ever included when K1_WITH_ONNX is defined (see
// module/Locomotion/CMakeLists.txt, which adds this source/header pair to
// the target's sources only under that option, and LocomotionController.cpp,
// which #ifdefs its own include of this header).
#ifdef K1_WITH_ONNX

#include <array>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include "module/Locomotion/src/LocomotionBackend.hpp"
#include "shared/k1/JointIndex.hpp"
#include "shared/sim/ModelMap.hpp"
#include "shared/sim/PdController.hpp"

namespace k1sim::module::backends {

// The policy controls the 12 leg joints only (JointIndexK1 10..21); arms are held
// at the ready pose and the head is driven by RotateHead. Contract (the single
// source of truth): mujoco/training_mjlab/OBS_ACTION_CONTRACT.md — 47-dim obs, 12-dim
// action, matching the proven gym base-walk format so a policy trained by
// mujoco/training_mjlab/ (mjlab) drops straight in.
inline constexpr std::size_t LEG_START = JointIndexK1::LeftHipPitch;  // 10
inline constexpr std::size_t LEG_COUNT = 12;                          // 10..21
inline constexpr std::size_t OBS_DIM   = 47;

// ONNX Runtime CPU policy backend. Every `inference_divisor` physics steps builds
// the 47-dim observation and runs the network for a fresh 12-dim leg action;
// between inferences, PD-tracks q_ref = ready + action_scale*action at 1 kHz. A
// gait clock (cos/sin of a phase advanced at `gait_frequency`) is part of the obs;
// when commanded speed is below `stand_threshold` the gait+command terms are zeroed
// (standing). Head handling matches KinematicBackend (RotateHead + gravity comp).
class PolicyBackend : public LocomotionBackend {
public:
    PolicyBackend(const ModelMap& map,
                  PdController pd,
                  std::array<double, JOINT_COUNT> ready_pose,
                  const YAML::Node& policy_cfg);

    void reset(const mjModel* m, mjData* d) override;
    void update(const mjModel* m, mjData* d, const LocoCommand& cmd) override;
    bool is_dynamic() const override {
        return true;  // root is real physics; only the joints are PD-driven
    }
    const char* name() const override {
        return "policy";
    }

private:
    std::array<float, OBS_DIM> build_obs(const mjModel* m, mjData* d, const LocoCommand& cmd) const;
    void run_inference(const mjModel* m, mjData* d, const LocoCommand& cmd);

    const ModelMap& map_;
    PdController pd_;
    std::array<double, JOINT_COUNT> ready_pose_;
    double action_scale_;
    double gait_frequency_;   // Hz, applied while moving
    double stand_threshold_;  // command speed below which the robot stands (gait_freq → 0)
    int inference_divisor_;

    int step_counter_    = 0;
    double gait_phase_   = 0.0;  // [0,1), advanced at gait_frequency each inference
    std::array<double, LEG_COUNT> last_action_{};

    Ort::Env env_;
    Ort::Session session_;
    Ort::MemoryInfo mem_info_;
};

}  // namespace k1sim::module::backends

#endif  // K1_WITH_ONNX

#endif  // K1SIM_MODULE_LOCOMOTION_BACKENDS_POLICYBACKEND_HPP
