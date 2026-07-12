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

// The policy controls all 22 joints (full-body, JointIndexK1 order); the head
// targets are overridden with the RotateHead command after inference. Contract
// (the single source of truth): docs/OBS_ACTION_CONTRACT.md — 82-dim obs, 22-dim
// action, matching the mujoco_playground K1Joystick* tasks so a policy trained
// by the NUbots mujoco_playground fork drops straight in.
inline constexpr std::size_t ACT_DIM = JOINT_COUNT;                        // 22
inline constexpr std::size_t OBS_DIM = 3 + 3 + 3 + 3 + 3 * ACT_DIM + 4;   // 82

// ONNX Runtime CPU policy backend. Every `inference_divisor` physics steps builds
// the 82-dim observation and runs the network for a fresh 22-dim action; between
// inferences, PD-tracks q_ref = default_pose + action_scale_joint*action at 1 kHz
// with the training-time gains (policy.kp/kd — NOT gains.yaml, which is tuned for
// the kinematic/stand controllers). A two-foot gait phase (cos/sin pairs advanced
// at `gait_frequency`) is part of the obs; when commanded speed is below
// `stand_threshold` the phase observation is pinned to [pi, pi] (standing).
// Head handling matches KinematicBackend (RotateHead + gravity comp).
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
    PdController pd_;  // gains overridden from policy.kp/kd (training-time gains)
    std::array<double, JOINT_COUNT> default_pose_;        // policy.default_pose (training keyframe)
    std::array<double, JOINT_COUNT> action_scale_joint_;  // policy.action_scale_joint * action_scale
    double gait_frequency_;   // Hz, phase advance rate
    double stand_threshold_;  // command norm below which the phase obs pins to [pi, pi]
    int inference_divisor_;

    int step_counter_ = 0;
    // Per-foot gait phase, anti-phase [0, pi] (reset() re-establishes this).
    std::array<double, 2> phase_{0.0, 3.141592653589793};
    std::array<double, ACT_DIM> last_action_{};

    Ort::Env env_;
    Ort::Session session_;
    Ort::MemoryInfo mem_info_;
};

}  // namespace k1sim::module::backends

#endif  // K1_WITH_ONNX

#endif  // K1SIM_MODULE_LOCOMOTION_BACKENDS_POLICYBACKEND_HPP
