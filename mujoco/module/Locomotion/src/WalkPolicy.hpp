#ifndef K1SIM_MODULE_LOCOMOTION_WALKPOLICY_HPP
#define K1SIM_MODULE_LOCOMOTION_WALKPOLICY_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "shared/k1/JointIndex.hpp"

namespace k1sim::module {

    // In-sim velocity-tracking walk policy: a plain MLP exported from an mjlab/AMP
    // K1 ONNX by scripts/export_walk_policy.py (JSON weights; no ONNX runtime needed).
    //
    //   obs (75) = base_ang_vel(3) projected_gravity(3) (q - default)(22) dq(22)
    //              last_action(22) command[vx,vy,vyaw](3)
    //   q_ref[j] = default[j] + action_scale[j] * action[j]
    //
    // Joint order is JointIndexK1 (identical to the training order). The policy is
    // stateful only through last_action. No NUClear / MuJoCo dependency so it is
    // directly unit-testable.
    class WalkPolicy {
    public:
        using JointArray = std::array<double, JOINT_COUNT>;

        // Throws std::runtime_error on a missing/malformed file.
        static std::unique_ptr<WalkPolicy> load(const std::string& path);

        void reset();

        // One inference. `gyro` body-frame rad/s, `gravity` = world (0,0,-1) in body frame.
        // Returns the joint position targets (clamping to joint limits is the caller's job).
        JointArray infer(const std::array<double, 3>& gyro,
                         const std::array<double, 3>& gravity,
                         const JointArray& q,
                         const JointArray& dq,
                         const std::array<double, 3>& command);

        const JointArray& default_pose() const {
            return default_pose_;
        }
        const JointArray& kp() const {
            return kp_;
        }
        const JointArray& kd() const {
            return kd_;
        }

        static constexpr std::size_t OBS_DIM = 75;

    private:
        struct Layer {
            std::size_t n_in = 0, n_out = 0;
            std::vector<float> w, b;  // w is [n_out, n_in] row-major
            enum class Act { Linear, Elu, Relu, Tanh } act = Act::Linear;
        };

        std::vector<Layer> layers_;
        std::array<float, OBS_DIM> obs_mean_{}, obs_std_{};
        JointArray default_pose_{}, action_scale_{}, kp_{}, kd_{};
        std::array<double, JOINT_COUNT> last_action_{};
    };

}  // namespace k1sim::module

#endif  // K1SIM_MODULE_LOCOMOTION_WALKPOLICY_HPP
