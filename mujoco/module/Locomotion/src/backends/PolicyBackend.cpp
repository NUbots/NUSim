#include "module/Locomotion/src/backends/PolicyBackend.hpp"

#ifdef K1_WITH_ONNX

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>

#include "shared/util/Config.hpp"

namespace k1sim::module::backends {

namespace {

constexpr double kHeadPitchMin = -0.3;
constexpr double kHeadPitchMax = 1.0;
constexpr double kHeadYawLimit = 0.785;
constexpr double kTwoPi        = 6.283185307179586;
constexpr double kDofVelScale  = 0.1;  // obs normalization on joint velocity (booster/htwk)

// Resolves policy.path (relative to the mujoco/ source root, like every other
// config path) and fatal-throws if the file is missing.
std::string resolve_and_validate_path(const YAML::Node& policy_cfg) {
    const std::string configured = policy_cfg["path"].as<std::string>("");
    if (configured.empty()) {
        throw std::runtime_error(
            "config/locomotion.yaml: backend: policy requires policy.path to point at an .onnx file");
    }
    const std::filesystem::path resolved = config::resolve_path(configured);
    if (!std::filesystem::exists(resolved)) {
        std::fprintf(stderr, "FATAL: PolicyBackend: policy.path '%s' (resolved '%s') does not exist\n",
                     configured.c_str(), resolved.string().c_str());
        throw std::runtime_error("PolicyBackend: onnx model file not found: " + resolved.string());
    }
    return resolved.string();
}

Ort::SessionOptions make_session_options() {
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    return opts;
}

}  // namespace

PolicyBackend::PolicyBackend(const ModelMap& map,
                              PdController pd,
                              std::array<double, JOINT_COUNT> ready_pose,
                              const YAML::Node& policy_cfg)
    : map_(map)
    , pd_(std::move(pd))
    , ready_pose_(ready_pose)
    , action_scale_(policy_cfg["action_scale"].as<double>(0.5))
    , gait_frequency_(policy_cfg["gait_frequency"].as<double>(1.5))
    , stand_threshold_(policy_cfg["stand_threshold"].as<double>(0.05))
    , inference_divisor_(std::max(1, policy_cfg["inference_divisor"].as<int>(20)))
    , env_(ORT_LOGGING_LEVEL_WARNING, "k1sim_policy_backend")
    , session_(env_, resolve_and_validate_path(policy_cfg).c_str(), make_session_options())
    , mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU)) {}

void PolicyBackend::reset(const mjModel* /*m*/, mjData* /*d*/) {
    step_counter_ = 0;  // forces an inference on the very next update()
    gait_phase_   = 0.0;
    last_action_.fill(0.0);
}

std::array<float, OBS_DIM> PolicyBackend::build_obs(const mjModel* /*m*/, mjData* d, const LocoCommand& cmd) const {
    std::array<float, OBS_DIM> obs{};
    std::size_t idx = 0;

    // [0:3] projected gravity: world "down" in the body frame (unit vector).
    const double* quat = &d->qpos[map_.root_qpos_adr + 3];  // wxyz, body->world
    double inv_quat[4];
    mju_negQuat(inv_quat, quat);
    const double down_world[3] = {0.0, 0.0, -1.0};
    double grav_body[3];
    mju_rotVecQuat(grav_body, down_world, inv_quat);
    obs[idx++] = static_cast<float>(grav_body[0]);
    obs[idx++] = static_cast<float>(grav_body[1]);
    obs[idx++] = static_cast<float>(grav_body[2]);

    // [3:6] base angular velocity (gyro), body frame.
    if (map_.sens_gyro >= 0) {
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_gyro + 0]);
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_gyro + 1]);
        obs[idx++] = static_cast<float>(d->sensordata[map_.sens_gyro + 2]);
    }
    else {
        idx += 3;
    }

    // [6:9] command, [9:11] gait clock — both zeroed while standing.
    const double speed  = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy + cmd.vyaw * cmd.vyaw);
    const bool moving   = speed > stand_threshold_;
    obs[idx++] = moving ? static_cast<float>(cmd.vx) : 0.0f;
    obs[idx++] = moving ? static_cast<float>(cmd.vy) : 0.0f;
    obs[idx++] = moving ? static_cast<float>(cmd.vyaw) : 0.0f;
    obs[idx++] = moving ? static_cast<float>(std::cos(kTwoPi * gait_phase_)) : 0.0f;
    obs[idx++] = moving ? static_cast<float>(std::sin(kTwoPi * gait_phase_)) : 0.0f;

    // [11:23] leg (q - default), [23:35] leg dq*scale.
    for (std::size_t k = 0; k < LEG_COUNT; ++k) {
        const std::size_t j = LEG_START + k;
        obs[idx++] = static_cast<float>(d->qpos[map_.qpos_adr[j]] - ready_pose_[j]);
    }
    for (std::size_t k = 0; k < LEG_COUNT; ++k) {
        const std::size_t j = LEG_START + k;
        obs[idx++] = static_cast<float>(d->qvel[map_.dof_adr[j]] * kDofVelScale);
    }
    // [35:47] previous action.
    for (std::size_t k = 0; k < LEG_COUNT; ++k) {
        obs[idx++] = static_cast<float>(last_action_[k]);
    }
    return obs;
}

void PolicyBackend::run_inference(const mjModel* m, mjData* d, const LocoCommand& cmd) {
    auto obs = build_obs(m, d, cmd);
    const std::array<int64_t, 2> obs_shape{1, static_cast<int64_t>(obs.size())};
    Ort::Value obs_tensor =
        Ort::Value::CreateTensor<float>(mem_info_, obs.data(), obs.size(), obs_shape.data(), obs_shape.size());

    static constexpr const char* kInputNames[]  = {"obs"};
    static constexpr const char* kOutputNames[] = {"action"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, kInputNames, &obs_tensor, 1, kOutputNames, 1);

    const float* action_data = outputs.front().GetTensorData<float>();
    for (std::size_t k = 0; k < LEG_COUNT; ++k) {
        last_action_[k] = static_cast<double>(action_data[k]);
    }
}

void PolicyBackend::update(const mjModel* m, mjData* d, const LocoCommand& cmd) {
    if (step_counter_ % inference_divisor_ == 0) {
        run_inference(m, d, cmd);
        // Advance the gait clock once per inference (50 Hz), at gait_frequency
        // while moving; frozen at 0 while standing so cos/sin stay well-defined.
        const double speed = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy + cmd.vyaw * cmd.vyaw);
        if (speed > stand_threshold_) {
            const double dt = static_cast<double>(inference_divisor_) * m->opt.timestep;
            gait_phase_     = std::fmod(gait_phase_ + dt * gait_frequency_, 1.0);
        }
        else {
            gait_phase_ = 0.0;
        }
    }
    ++step_counter_;

    // Base target = ready pose (holds arms + torso); overwrite the 12 legs with
    // the policy action and the head with the RotateHead command.
    std::array<double, JOINT_COUNT> q_ref = ready_pose_;
    for (std::size_t k = 0; k < LEG_COUNT; ++k) {
        q_ref[LEG_START + k] = ready_pose_[LEG_START + k] + action_scale_ * last_action_[k];
    }
    q_ref[JointIndexK1::HeadPitch] = std::clamp(cmd.head_pitch, kHeadPitchMin, kHeadPitchMax);
    q_ref[JointIndexK1::HeadYaw]   = std::clamp(cmd.head_yaw, -kHeadYawLimit, kHeadYawLimit);

    std::array<double, JOINT_COUNT> dq_ref{};
    std::array<double, JOINT_COUNT> tau_ff{};
    tau_ff[JointIndexK1::HeadPitch] = d->qfrc_bias[map_.dof_adr[JointIndexK1::HeadPitch]];
    tau_ff[JointIndexK1::HeadYaw]   = d->qfrc_bias[map_.dof_adr[JointIndexK1::HeadYaw]];

    pd_.apply(m, d, map_, q_ref, dq_ref, tau_ff);
}

}  // namespace k1sim::module::backends

#endif  // K1_WITH_ONNX
